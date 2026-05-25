import os
import time
import sys
import logging
import threading
import requests
import numpy as np
import cv2
from bs4 import BeautifulSoup
from typing import List, Optional, Any

try:
    import torch
    from torchvision import transforms
    torch_available = True
except ImportError:
    torch_available = False

# Configuration for the default lane detection model
MODEL_CONFIG = {
    "HF_OWNER": "Tumppi066",
    "HF_REPOSITORY": "ets2la-models",
    "HF_FOLDER": "lane-detection",
}

class Model:
    """
    Handles loading and inference of PyTorch models for lane detection.
    Ported and adapted from the original ETS2LA project.
    """
    def __init__(
        self,
        HF_owner: str = MODEL_CONFIG["HF_OWNER"],
        HF_repository: str = MODEL_CONFIG["HF_REPOSITORY"],
        HF_model_folder: str = MODEL_CONFIG["HF_FOLDER"],
        torch_dtype=torch.bfloat16 if torch_available else None,
        threaded: bool = True,
    ):
        self.torch_dtype = torch_dtype
        self.device = torch.device("cuda" if torch_available and torch.cuda.is_available() else "cpu")

        # Path to the model cache
        # Assumes the project root is the current working directory or parent of 'core'
        self.base_path = os.path.dirname(os.path.realpath(__file__))
        self.path = os.path.join(self.base_path, "..", "model-cache", HF_owner, HF_repository, HF_model_folder)
        self.path = os.path.abspath(self.path)

        self.HF_owner = str(HF_owner)
        self.HF_repository = str(HF_repository)
        self.HF_model_folder = str(HF_model_folder)
        self.identifier = f"{HF_repository}/{HF_model_folder}"

        self.threaded = threaded
        self.update_thread: Optional[threading.Thread] = None
        self.load_thread: Optional[threading.Thread] = None
        self.loaded = False

        self.metadata: dict = {}
        self.model: Any = None
        self.image_width: int = 0
        self.image_height: int = 0
        self.color_channels: int = 0
        self.outputs: int = 0

    def detect(self, image: np.ndarray) -> List:
        """Run the model on an image.
        Automatically converts and resizes the image.
        """
        if not self.loaded or not torch_available:
            return []

        try:
            # Preprocessing
            if len(image.shape) == 3:
                if image.shape[2] == 4 and self.color_channels == 3:
                    image = cv2.cvtColor(image, cv2.COLOR_RGBA2RGB)
                elif image.shape[2] == 1 and self.color_channels == 3:
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)
                elif image.shape[2] == 3 and self.color_channels == 1:
                    image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
            elif len(image.shape) == 2:
                if self.color_channels == 3:
                    image = cv2.cvtColor(image, cv2.COLOR_GRAY2RGB)

            image = cv2.resize(image, (self.image_width, self.image_height))
            if image.dtype == np.uint8:
                image = np.array(image, dtype=np.float32) / 255.0

            image = torch.as_tensor(
                transforms.ToTensor()(image).unsqueeze(0),
                dtype=self.torch_dtype,
                device=self.device,
            )

            with torch.no_grad():
                output = self.model(image)
                output = output.tolist()
            return output
        except Exception as e:
            logging.error(f"PyTorch - Error in function detect [{self.identifier}]: {e}")
            return []

    def load_model(self):
        """Load the model from the cache, automatically handles updates."""
        if not torch_available:
            logging.error(f"PyTorch not available. Skipping model load for {self.identifier}")
            return

        def thread():
            try:
                self.check_for_updates()
                if self.update_thread and self.update_thread.is_alive():
                    self.update_thread.join()

                model_file = self.get_name()
                if model_file is None:
                    logging.warning(f"[{self.identifier}] No model file found to load.")
                    return

                logging.info(f"[{self.identifier}] Loading the model...")

                try:
                    # Load JIT model and metadata
                    meta_storage = {}
                    self.model = torch.jit.load(
                        os.path.join(self.path, model_file),
                        _extra_files=meta_storage,
                        map_location=self.device,
                    )
                    self.model.eval()
                    self.model.to(self.torch_dtype)

                    # Extract metadata from the storage
                    # The reference project used an eval() on a string in the metadata,
                    # which is risky. We'll try to find the key with the most content.
                    key = max(meta_storage, key=lambda k: len(meta_storage[k])) if meta_storage else None
                    if key:
                        try:
                            self.metadata = eval(meta_storage[key])
                            for item in self.metadata:
                                item = str(item)
                                if "image_width" in item.lower(): self.image_width = int(item.split("#")[1])
                                if "image_height" in item.lower(): self.image_height = int(item.split("#")[1])
                                if "image_channels" in item.lower() or "color_channels" in item.lower():
                                    val = item.split("#")[1]
                                    try: self.color_channels = int(val)
                                    except ValueError: pass
                                if "outputs" in item.lower(): self.outputs = int(item.split("#")[1])
                        except Exception as e:
                            logging.warning(f"[{self.identifier}] Unable to parse model metadata: {e}")

                    self.loaded = True
                    logging.info(f"[{self.identifier}] Successfully loaded the model!")
                except Exception as e:
                    logging.error(f"[{self.identifier}] Failed to load model file: {e}")
                    self.loaded = False
                    self.handle_broken()

            except Exception as e:
                logging.error(f"PyTorch - Error in load_model thread: {e}")
                self.loaded = False

        if self.threaded:
            self.load_thread = threading.Thread(target=thread, daemon=True)
            self.load_thread.start()
        else:
            thread()

    def check_for_updates(self):
        """Checks for model updates on Hugging Face."""
        if not torch_available: return

        def thread():
            try:
                if self.get_name() is not None and not ("--dev" in sys.argv):
                    # Minimal check for updates could be implemented here.
                    # For now, we'll focus on downloading if missing.
                    pass

                if not os.path.exists(self.path):
                    os.makedirs(self.path, exist_ok=True)

                if self.get_name() is None:
                    logging.info(f"[{self.identifier}] Model not found, downloading from Hugging Face...")
                    self._download_latest()

            except Exception as e:
                logging.error(f"PyTorch - Error in check_for_updates: {e}")

        if self.threaded:
            self.update_thread = threading.Thread(target=thread, daemon=True)
            self.update_thread.start()
        else:
            thread()

    def _download_latest(self):
        """Downloads the latest .pt model from Hugging Face."""
        try:
            url = f"https://huggingface.co/{self.HF_owner}/{self.HF_repository}/tree/main/{self.HF_model_folder}"
            response = requests.get(url)
            soup = BeautifulSoup(response.content, "html.parser")

            latest_model = None
            for link in soup.find_all("a", href=True):
                href = link["href"]
                if href.startswith(f"/{self.HF_owner}/{self.HF_repository}/blob/main/{self.HF_model_folder}"):
                    latest_model = href.split("/")[-1]
                    break

            if latest_model:
                download_url = f"https://huggingface.co/{self.HF_owner}/{self.HF_repository}/resolve/main/{self.HF_model_folder}/{latest_model}?download=true"
                logging.info(f"[{self.identifier}] Downloading {latest_model}...")

                res = requests.get(download_url, stream=True)
                with open(os.path.join(self.path, latest_model), "wb") as f:
                    for chunk in res.iter_content(chunk_size=1024):
                        f.write(chunk)
                logging.info(f"[{self.identifier}] Download complete.")
            else:
                logging.error(f"[{self.identifier}] Could not find model file on Hugging Face.")
        except Exception as e:
            logging.error(f"[{self.identifier}] Failed to download model: {e}")

    def get_name(self) -> Optional[str]:
        """Returns the name of the first .pt file in the cache."""
        try:
            if not os.path.exists(self.path): return None
            for file in os.listdir(self.path):
                if file.endswith(".pt"):
                    return file
            return None
        except Exception:
            return None

    def delete(self):
        """Deletes the local model cache."""
        try:
            if os.path.exists(self.path):
                for file in os.listdir(self.path):
                    if file.endswith(".pt"):
                        os.remove(os.path.join(self.path, file))
        except Exception as e:
            logging.error(f"[{self.identifier}] Failed to delete model: {e}")

    def handle_broken(self):
        """Deletes broken model and attempts redownload."""
        self.delete()
        self.check_for_updates()
        # Wait for download thread if it started
        if self.update_thread and self.update_thread.is_alive():
            self.update_thread.join()
        self.load_model()
