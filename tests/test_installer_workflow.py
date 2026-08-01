import io
import json
import os
import zipfile

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PyQt6.QtWidgets import QApplication
from PyQt6.QtTest import QTest

import installer


def test_release_version_is_042_everywhere():
    from core import update_check
    assert installer.APP_VERSION == "0.4.2"
    assert update_check.VERSION == installer.APP_VERSION
    freeze_source = (os.path.join(os.path.dirname(installer.__file__),
                                  "freeze_app.py"))
    with open(freeze_source, encoding="utf-8") as stream:
        assert 'VERSION = "0.4.2"' in stream.read()


def test_pip_install_fails_closed_and_reports_exact_required_package(
        tmp_path, monkeypatch):
    root = tmp_path / "install"
    root.mkdir()
    (root / "requirements.txt").write_text(
        "requests\nvgamepad>=0.1.0\n", encoding="utf-8")
    worker = installer.InstallWorker(str(root), "sk")
    messages = []
    worker.log.connect(messages.append)
    monkeypatch.setattr(worker, "_real_python", lambda: ["python"])

    class Result:
        def __init__(self, code, stderr=""):
            self.returncode = code
            self.stderr = stderr
            self.stdout = ""

    def fake_run(command, **_kwargs):
        package = command[-1]
        return Result(1, "No matching distribution") if package.startswith(
            "vgamepad") else Result(0)

    monkeypatch.setattr(installer.subprocess, "run", fake_run)
    assert worker._pip_install() is False
    assert any("vgamepad>=0.1.0" in line and "No matching distribution" in line
               for line in messages)


def test_pip_install_requires_every_package_to_succeed(tmp_path, monkeypatch):
    root = tmp_path / "install"
    root.mkdir()
    (root / "requirements.txt").write_text("vgamepad>=0.1.0\n", encoding="utf-8")
    worker = installer.InstallWorker(str(root), "sk")
    monkeypatch.setattr(worker, "_real_python", lambda: ["python"])

    class Result:
        returncode = 0
        stderr = ""
        stdout = ""

    monkeypatch.setattr(installer.subprocess, "run",
                        lambda *_args, **_kwargs: Result())
    assert worker._pip_install() is True


@pytest.fixture(scope="module")
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


@pytest.mark.parametrize(
    "path",
    [
        "main.py",
        "bootloader.py",
        "requirements.txt",
        "assets/favicon.ico",
        "core/navigation/route.py",
        "languages/sk.json",
        "plugins/map/main.py",
        "sdk/plugin_sdk.py",
        "ui/app.py",
        "UI/icons.py",
    ],
)
def test_runtime_payload_allowlist_accepts_only_application_files(path):
    assert installer._is_runtime_payload_path(path)


@pytest.mark.parametrize(
    "path",
    [
        "installer.py",
        "build_installer.py",
        "freeze_app.py",
        "tests/test_lane_model.py",
        ".codex/config.toml",
        ".claude/settings.json",
        ".zcode/config.json",
        ".agents/AGENTS.md",
        "docs/design.md",
        "tools/generate.py",
        "settings.json",
        "ultrapilot.log",
        "UltraPilot_Installer.spec",
        "CHANGELOG.md",
        "core/__pycache__/route.pyc",
    ],
)
def test_runtime_payload_allowlist_rejects_development_and_user_files(path):
    assert not installer._is_runtime_payload_path(path)


def test_copy_runtime_tree_does_not_copy_repository_tooling(tmp_path):
    source = tmp_path / "source"
    target = tmp_path / "target"
    files = {
        "main.py": "run",
        "core/engine.py": "engine",
        "assets/favicon.ico": "icon",
        "tests/test_engine.py": "test",
        ".codex/config.toml": "codex",
        "installer.py": "installer",
    }
    for rel, content in files.items():
        path = source / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content, encoding="utf-8")

    copied = installer._copy_runtime_tree(str(source), str(target))

    assert copied == 3
    assert (target / "main.py").is_file()
    assert (target / "core" / "engine.py").is_file()
    assert not (target / "tests").exists()
    assert not (target / ".codex").exists()
    assert not (target / "installer.py").exists()


def test_zip_strategy_extracts_runtime_payload_only(tmp_path, monkeypatch):
    archive = io.BytesIO()
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("ets2la-main/main.py", "print('ok')")
        zf.writestr("ets2la-main/bootloader.py", "boot = True")
        zf.writestr("ets2la-main/requirements.txt", "requests")
        zf.writestr("ets2la-main/core/engine.py", "engine = True")
        zf.writestr("ets2la-main/assets/data.bin", b"x" * 2048)
        zf.writestr("ets2la-main/tests/test_engine.py", "bad")
        zf.writestr("ets2la-main/.codex/config.toml", "bad")
        zf.writestr("ets2la-main/installer.py", "bad")
    payload = archive.getvalue()

    class Response:
        status_code = 200
        headers = {"Content-Length": str(len(payload))}

        @staticmethod
        def iter_content(chunk_size=65536):
            yield payload

    import requests
    monkeypatch.setattr(requests, "get", lambda *args, **kwargs: Response())
    worker = installer.InstallWorker(str(tmp_path / "install"), "sk")

    assert worker._try_zip_archive()
    root = tmp_path / "install"
    assert (root / "main.py").is_file()
    assert (root / "core" / "engine.py").is_file()
    assert not (root / "tests").exists()
    assert not (root / ".codex").exists()
    assert not (root / "installer.py").exists()


def test_zip_strategy_discards_partial_requests_bytes_before_urllib_fallback(
        tmp_path, monkeypatch):
    archive = io.BytesIO()
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("ets2la-main/main.py", "print('ok')")
        zf.writestr("ets2la-main/bootloader.py", "boot = True")
        zf.writestr("ets2la-main/requirements.txt", "requests")
        zf.writestr("ets2la-main/assets/data.bin", b"x" * 2048)
    payload = archive.getvalue()

    class PartialResponse:
        status_code = 200
        headers = {"Content-Length": str(len(payload))}
        closed = False

        def iter_content(self, chunk_size=65536):
            yield b"PK\x03\x04partial"
            raise ConnectionError("connection reset")

        def close(self):
            self.closed = True

    partial = PartialResponse()

    class UrlResponse:
        status = 200
        headers = {"Content-Length": str(len(payload))}

        def __init__(self):
            self._sent = False

        def __enter__(self):
            return self

        def __exit__(self, *_args):
            return False

        def read(self, _size):
            if self._sent:
                return b""
            self._sent = True
            return payload

    import requests
    import urllib.request
    monkeypatch.setattr(requests, "get", lambda *args, **kwargs: partial)
    monkeypatch.setattr(urllib.request, "urlopen",
                        lambda *args, **kwargs: UrlResponse())
    worker = installer.InstallWorker(str(tmp_path / "install"), "sk")

    assert worker._try_zip_archive()
    assert partial.closed
    assert (tmp_path / "install" / "main.py").is_file()


def test_zip_strategy_tries_second_endpoint_when_first_archive_is_invalid(
        tmp_path, monkeypatch):
    archive = io.BytesIO()
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("ets2la-main/main.py", "print('ok')")
        zf.writestr("ets2la-main/bootloader.py", "boot = True")
        zf.writestr("ets2la-main/requirements.txt", "requests")
        zf.writestr("ets2la-main/core/engine.py", "ready = True")
        zf.writestr("ets2la-main/assets/data.bin", b"x" * 2048)
    payload = archive.getvalue()
    calls = []

    class Response:
        status_code = 200
        headers = {}

        def __init__(self, body):
            self.body = body

        def iter_content(self, chunk_size=65536):
            yield self.body

        def close(self):
            pass

    import requests

    def fake_get(url, **_kwargs):
        calls.append(url)
        return Response(b"not-a-zip" * 256 if len(calls) == 1 else payload)

    monkeypatch.setattr(requests, "get", fake_get)
    worker = installer.InstallWorker(str(tmp_path / "install"), "sk")

    assert worker._try_zip_archive()
    assert calls == [installer.CODELOAD_URL, installer.ARCHIVE_URL]
    assert (tmp_path / "install" / "core" / "engine.py").is_file()


def test_zip_strategy_rejects_partially_extracted_runtime(tmp_path, monkeypatch):
    archive = io.BytesIO()
    with zipfile.ZipFile(archive, "w") as zf:
        zf.writestr("ets2la-main/main.py", "print('ok')")
        zf.writestr("ets2la-main/bootloader.py", "boot = True")
        zf.writestr("ets2la-main/requirements.txt", "requests")
        zf.writestr("ets2la-main/core/engine.py", "ready = True")
        zf.writestr("ets2la-main/assets/data.bin", b"x" * 2048)
    payload = archive.getvalue()

    class Response:
        status_code = 200
        headers = {"Content-Length": str(len(payload))}

        @staticmethod
        def iter_content(chunk_size=65536):
            yield payload

        @staticmethod
        def close():
            pass

    import requests
    monkeypatch.setattr(requests, "get", lambda *args, **kwargs: Response())
    blocked_file = tmp_path / "install" / "core" / "engine.py"
    blocked_file.mkdir(parents=True)
    worker = installer.InstallWorker(str(tmp_path / "install"), "sk")

    assert not worker._try_zip_archive()
    assert (tmp_path / "install" / "main.py").is_file()
    assert blocked_file.is_dir()


def test_fetch_prefers_filtered_raw_runtime_transport(tmp_path, monkeypatch):
    worker = installer.InstallWorker(str(tmp_path / "install"), "sk")
    calls = []
    monkeypatch.setattr(worker, "_try_raw_file_by_file",
                        lambda: calls.append("raw") or True)
    monkeypatch.setattr(worker, "_try_git_clone",
                        lambda: calls.append("git") or True)
    monkeypatch.setattr(worker, "_try_zip_archive",
                        lambda: calls.append("zip") or True)
    monkeypatch.setattr(worker, "_finalise_runtime_payload",
                        lambda: calls.append("finalise"))

    assert worker._fetch_repo()
    assert calls == ["raw", "finalise"]


def test_cleanup_removes_only_legacy_development_payload(tmp_path):
    root = tmp_path / "UltraPilot"
    (root / "main.py").parent.mkdir(parents=True)
    (root / "main.py").write_text("run", encoding="utf-8")
    for rel in ("tests/test_x.py", ".claude/config.json", "build/file.bin"):
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("dev", encoding="utf-8")
    for rel in ("settings.json", "routes/mine.json", "ultrapilot.log"):
        path = root / rel
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("user", encoding="utf-8")

    removed = installer._remove_legacy_development_payload(str(root))

    assert {"tests", ".claude", "build"}.issubset(set(removed))
    assert (root / "settings.json").is_file()
    assert (root / "routes" / "mine.json").is_file()
    assert (root / "ultrapilot.log").is_file()


def test_sdk_target_normalisation_supports_all_record_versions(tmp_path):
    game = tmp_path / "Euro Truck Simulator 2"
    plugins = game / "bin" / "win_x64" / "plugins"
    expected = os.path.normpath(str(plugins))

    assert installer._sdk_plugins_dir(str(game)) == expected
    assert installer._sdk_plugins_dir(str(game / "bin")) == expected
    assert installer._sdk_plugins_dir(str(game / "bin" / "win_x64")) == expected
    assert installer._sdk_plugins_dir(str(plugins)) == expected
    assert installer._sdk_game_root(str(plugins)) == os.path.normpath(str(game))


@pytest.mark.parametrize("record_form", ["root", "legacy_bin", "plugins"])
def test_sdk_uninstall_removes_dlls_for_current_and_legacy_records(
        tmp_path, monkeypatch, record_form):
    game = tmp_path / record_form / "Euro Truck Simulator 2"
    plugins = game / "bin" / "win_x64" / "plugins"
    plugins.mkdir(parents=True)
    for dll in installer._SDK_DLL_NAMES:
        (plugins / dll).write_bytes(b"dll")
    values = {
        "root": game,
        "legacy_bin": game / "bin",
        "plugins": plugins,
    }
    monkeypatch.setattr("core.sdk.game_utils.find_scs_games", lambda: [])

    result = installer._do_uninstall_sdk({"sdk_targets": [str(values[record_form])]})

    assert result == {"removed": 3, "failed": 0, "found": 3}
    assert not any((plugins / dll).exists() for dll in installer._SDK_DLL_NAMES)


def test_sdk_repair_persists_canonical_roots_atomically(tmp_path):
    game = tmp_path / "Euro Truck Simulator 2"
    plugins = game / "bin" / "win_x64" / "plugins"
    record_path = tmp_path / "install.json"
    record = {"install_path": "C:/UltraPilot", "sdk_targets": []}

    roots = installer._update_sdk_targets(record, [str(plugins)], str(record_path))

    assert roots == [os.path.normpath(str(game))]
    assert json.loads(record_path.read_text(encoding="utf-8"))["sdk_targets"] == roots
    assert not (tmp_path / "install.json.tmp").exists()


def test_onboarding_sdk_repair_keeps_all_required_dlls_and_uninstall_removes_them(
        tmp_path, monkeypatch):
    from core.sdk import sdk_downloader

    assets = tmp_path / "assets"
    game = tmp_path / "Euro Truck Simulator 2"
    assets.mkdir()
    game.mkdir()
    for index, dll in enumerate(sdk_downloader.SDK_FILES):
        (assets / dll).write_bytes(("new-{}".format(index)).encode("ascii"))
    monkeypatch.setattr(sdk_downloader, "_dll_source_dir", lambda: str(assets))

    ok, message = sdk_downloader.ensure_installed(str(game), "1.59")
    plugins = game / "bin" / "win_x64" / "plugins"
    assert (ok, message) == (True, "installed")
    assert sdk_downloader.is_sdk_installed(str(game))
    assert all((plugins / dll).is_file() for dll in sdk_downloader.SDK_FILES)

    # Repair must replace every required file, including ets2la_plugin.dll.
    for dll in sdk_downloader.SDK_FILES:
        (plugins / dll).write_bytes(b"damaged")
    ok, message = sdk_downloader.repair(str(game), "1.59")
    assert (ok, message) == (True, "installed")
    assert all((plugins / dll).read_bytes() != b"damaged"
               for dll in sdk_downloader.SDK_FILES)

    ok, message = sdk_downloader.uninstall(str(game))
    assert (ok, message) == (True, "uninstalled")
    assert not any((plugins / dll).exists() for dll in sdk_downloader.SDK_FILES)


def test_welcome_page_contains_centred_static_intro_and_language_card(qapp):
    window = installer.InstallerWindow(lang="sk", theme="light")
    try:
        assert window.size().width() == 920
        assert window.size().height() == 700
        assert not (window.windowFlags()
                    & installer.Qt.WindowType.FramelessWindowHint)
        assert window.windowFlags() & installer.Qt.WindowType.WindowTitleHint
        assert window.windowFlags() & installer.Qt.WindowType.WindowCloseButtonHint
        assert window.windowFlags() & installer.Qt.WindowType.WindowMinMaxButtonsHint
        assert window.lang_combo.count() == 2
        assert window.lang_combo.currentData() == "sk"
        assert "UltraPilot " + installer.APP_VERSION in window.ver_lbl.text()
        assert "Inštalátor " + installer.INSTALLER_VERSION in window.ver_lbl.text()
        assert "commit" not in window.ver_lbl.text().lower()
        assert not hasattr(window, "welcome_visual")
        assert (window.welcome_title.alignment()
                & installer.Qt.AlignmentFlag.AlignHCenter)
        assert (window.welcome_description.alignment()
                & installer.Qt.AlignmentFlag.AlignHCenter)
        assert isinstance(window.language_mark, installer.InstallerLanguageIcon)
        assert (window.language_mark.width(), window.language_mark.height()) == (38, 38)
        assert window.language_mark.accessibleName() == "Language"
        window.theme = "dark"
        window._apply_theme()
        qapp.processEvents()
    finally:
        window.close()

def test_installer_has_no_custom_control_dots_and_step_badges_are_unclipped(qapp):
    window = installer.InstallerWindow(lang="sk", theme="light")
    try:
        assert not hasattr(window, "window_controls")
        badges = [badge for badge, _label, _cell in window._step_labels]
        assert all(isinstance(badge, installer.InstallerStepBadge)
                   for badge in badges)
        assert [(badge.width(), badge.height()) for badge in badges] == [(32, 32)] * 5
        assert badges[0]._state == "active"
    finally:
        window.close()


def test_installer_theme_toggle_changes_theme_from_real_mouse_click(qapp):
    window = installer.InstallerWindow(lang="sk", theme="light")
    window.show()
    try:
        qapp.processEvents()
        light_sheet = window.styleSheet()
        assert window.theme == "light"
        assert not window.theme_btn.is_dark()

        QTest.mouseClick(window.theme_btn, installer.Qt.MouseButton.LeftButton)
        QTest.qWait(30)
        assert window.theme == "dark"
        assert window.theme_btn.is_dark()
        assert window.styleSheet() != light_sheet

        QTest.mouseClick(window.theme_btn, installer.Qt.MouseButton.LeftButton)
        QTest.qWait(30)
        assert window.theme == "light"
        assert not window.theme_btn.is_dark()
    finally:
        window.close()


def test_completed_install_can_revisit_step_four_log_and_return_to_finish(qapp):
    class FinishedWorker:
        @staticmethod
        def isRunning():
            return False

    window = installer.InstallerWindow(lang="sk", theme="light")
    try:
        window._worker = FinishedWorker()
        window.exe_path = "C:/UltraPilot/main.py"
        window.log_view.setPlainText("status log installation")
        window._go_step(4)

        window._step_labels[3][2].clicked.emit(3)
        assert window.stack.currentIndex() == 3
        assert "status log installation" in window.log_view.toPlainText()

        window._step_labels[4][2].clicked.emit(4)
        assert window.stack.currentIndex() == 4
    finally:
        window.close()
