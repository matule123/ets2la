import io
import os
import shutil
import unittest
import uuid
import zipfile
from unittest import mock

from PyQt6.QtTest import QTest
from PyQt6.QtWidgets import QApplication

from core import update_check
from UI.splash import BootSplash
from UI.update_widget import UpdateCheckerWidget, UpdateConfirmDialog


def update_archive():
    data = io.BytesIO()
    with zipfile.ZipFile(data, "w") as archive:
        archive.writestr("ets2la-main/core/new_module.py", "new = True")
        archive.writestr("ets2la-main/routes/user.json", "must not replace")
    return data.getvalue()


class StreamingResponse:
    status_code = 200

    def __init__(self, data, reported_length=None):
        self.data = data
        self.headers = {"Content-Length": str(
            len(data) if reported_length is None else reported_length)}

    def iter_content(self, chunk_size=128 * 1024):
        for offset in range(0, len(self.data), max(1, chunk_size // 3)):
            yield self.data[offset:offset + max(1, chunk_size // 3)]


class WorkspaceDirectory:
    def __enter__(self):
        self.path = os.path.join(
            os.path.dirname(__file__), ".update-workflow-" + uuid.uuid4().hex)
        os.makedirs(self.path)
        return self.path

    def __exit__(self, *_):
        shutil.rmtree(self.path, ignore_errors=True)


class UpdateStagingTests(unittest.TestCase):
    def test_download_reports_mb_and_does_not_install_until_confirmed(self):
        with WorkspaceDirectory() as root:
            cache = os.path.join(root, "cache")
            app = os.path.join(root, "app")
            os.makedirs(os.path.join(app, "routes"))
            user_route = os.path.join(app, "routes", "user.json")
            with open(user_route, "w", encoding="utf-8") as stream:
                stream.write("keep")
            progress = []
            data = update_archive()
            with (mock.patch.object(update_check, "_update_cache_dir",
                                    return_value=cache),
                  mock.patch.object(update_check, "_app_dir", return_value=app),
                  mock.patch("requests.get",
                             return_value=StreamingResponse(data))):
                self.assertTrue(update_check.prepare_update(
                    progress_cb=lambda fraction, text:
                    progress.append((fraction, text)),
                    target_commit="abcdef0"))
                self.assertFalse(os.path.exists(
                    os.path.join(app, "core", "new_module.py")))
                self.assertTrue(update_check.prepared_update_info())
                info = update_check.prepared_update_info()
                self.assertEqual(info["archive_bytes"], len(data))
                self.assertGreater(info["unpacked_bytes"], 0)
                self.assertEqual(info["file_count"], 2)
                self.assertIn("Pripravené na inštaláciu:", progress[-1][1])
                self.assertIn("stiahnutý balík", progress[-1][1])
                self.assertEqual(progress[-1][0], 1.0)

                self.assertTrue(update_check.install_prepared_update())
                self.assertTrue(os.path.isfile(
                    os.path.join(app, "core", "new_module.py")))
                with open(user_route, "r", encoding="utf-8") as stream:
                    self.assertEqual(stream.read(), "keep")
                self.assertFalse(update_check.prepared_update_info())
                self.assertTrue(update_check.take_update_startup_notice())
                self.assertFalse(update_check.take_update_startup_notice())

    def test_download_is_pinned_to_advertised_commit_and_not_cached_main(self):
        with WorkspaceDirectory() as root:
            response = StreamingResponse(update_archive())
            with (mock.patch.object(update_check, "_update_cache_dir",
                                    return_value=root),
                  mock.patch("requests.get", return_value=response) as request):
                self.assertTrue(update_check.prepare_update(
                    target_commit="abcdef0"))
            args, kwargs = request.call_args
            self.assertEqual(
                args[0],
                "https://github.com/matule123/ets2la/archive/abcdef0.zip")
            self.assertEqual(kwargs["headers"]["Cache-Control"], "no-cache")
            self.assertEqual(kwargs["headers"]["Accept-Encoding"], "identity")

    def test_download_progress_distinguishes_moving_bytes_from_total(self):
        mib = 1024 * 1024
        self.assertEqual(
            update_check._format_download_progress(mib, 4 * mib),
            "Stiahnuté 1.00 MB z 4.00 MB (25 %)")
        self.assertEqual(
            update_check._format_download_progress(mib, 0),
            "Stiahnuté 1.00 MB • celkovú veľkosť zisťujem")

    def test_verified_size_replaces_incorrect_http_content_length(self):
        with WorkspaceDirectory() as root:
            data = update_archive()
            progress = []
            response = StreamingResponse(data, reported_length=800 * 1024)
            with (mock.patch.object(update_check, "_update_cache_dir",
                                    return_value=root),
                  mock.patch("requests.get", return_value=response)):
                self.assertTrue(update_check.prepare_update(
                    progress_cb=lambda fraction, text:
                    progress.append((fraction, text)),
                    target_commit="abcdef0"))
                info = update_check.prepared_update_info()
                self.assertEqual(info["total_bytes"], len(data))
                self.assertEqual(info["archive_bytes"], len(data))
                self.assertNotIn("0.78 MB", progress[-1][1])
                self.assertEqual(
                    progress[-1][1],
                    update_check._format_prepared_update_size(info))

    def test_failed_download_never_creates_ready_manifest(self):
        response = mock.Mock(status_code=503, headers={})
        with (WorkspaceDirectory() as root,
              mock.patch.object(update_check, "_update_cache_dir",
                                return_value=root),
              mock.patch("requests.get", return_value=response)):
            self.assertFalse(update_check.prepare_update(target_commit="abcdef0"))
            self.assertEqual(update_check.prepared_update_info(), {})

    def test_old_manifest_is_enriched_from_existing_verified_zip(self):
        with WorkspaceDirectory() as root:
            archive_path = os.path.join(root, "update.zip")
            manifest_path = os.path.join(root, "update.json")
            data = update_archive()
            with open(archive_path, "wb") as stream:
                stream.write(data)
            with open(manifest_path, "w", encoding="utf-8") as stream:
                stream.write(
                    '{"target_commit":"abcdef0","total_bytes":838861}')
            with mock.patch.object(update_check, "_update_cache_dir",
                                   return_value=root):
                info = update_check.prepared_update_info()
            self.assertEqual(info["archive_bytes"], len(data))
            self.assertEqual(info["total_bytes"], len(data))
            self.assertGreater(info["unpacked_bytes"], 0)
            self.assertEqual(info["file_count"], 2)


class UpdateUiStateTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.app = QApplication.instance() or QApplication([])

    def test_dialog_has_explicit_download_progress_and_install_states(self):
        dialog = UpdateConfirmDialog(
            "abcdef0", title="Opravené riadenie",
            description="Podrobný zoznam zmien patrí iba do hover karty.")
        self.assertEqual(dialog.primary_btn.text(), "Stiahnuť")
        self.assertFalse(dialog.progress.isVisible())
        self.assertIsNotNone(dialog.brand_logo.pixmap())
        self.assertFalse(dialog.brand_logo.pixmap().isNull())
        self.assertNotIn("Opravené riadenie", dialog.note.text())
        self.assertNotIn("Podrobný zoznam zmien", dialog.note.text())

        dialog.set_downloading()
        self.assertEqual(
            dialog.progress_text.text(),
            "Stiahnuté 0.00 MB • celkovú veľkosť zisťujem")
        dialog.set_progress(0.5, "5.0 MB / 10.0 MB")
        self.assertEqual(dialog.progress.value(), 50)
        self.assertEqual(dialog.progress_percent.text(), "50 %")
        self.assertEqual(dialog.progress_text.text(), "5.0 MB / 10.0 MB")
        self.assertFalse(dialog.primary_btn.isEnabled())
        self.assertIn("#2563EB", dialog.progress.styleSheet())
        self.assertIn("#60A5FA", dialog.progress.styleSheet())
        self.assertNotIn("#10B981", dialog.progress.styleSheet())

        dialog.set_ready(
            "Pripravené na inštaláciu: 3.42 MB • stiahnutý balík 0.84 MB")
        self.assertEqual(
            dialog.title_lbl.text(),
            "Aktualizácia je pripravená na inštaláciu")
        self.assertEqual(dialog.primary_btn.text(),
                         "Inštalovať a reštartovať")
        self.assertEqual(
            dialog.progress_text.text(),
            "Pripravené na inštaláciu: 3.42 MB • stiahnutý balík 0.84 MB")
        self.assertTrue(dialog.primary_btn.isEnabled())
        self.assertEqual(dialog.progress_percent.text(), "100 %")
        self.assertEqual(dialog.phase_badge.text(), "Overené a pripravené")
        dialog.close()

    def test_available_update_uses_plain_update_button(self):
        with (mock.patch.object(update_check, "git_commit",
                                return_value="1234567"),
              mock.patch.object(update_check, "latest_commit_info",
                                return_value={"title": "Opravy",
                                              "description": ""})):
            widget = UpdateCheckerWidget(object())
            widget._on_checked(True, "abcdef0")
            self.assertEqual(widget.btn.text(), "Aktualizovať")
            widget.close()

    def test_release_changes_appear_only_in_hover_card_below_button(self):
        with (mock.patch.object(update_check, "git_commit",
                                return_value="1234567"),
              mock.patch.object(update_check, "latest_commit_info",
                                return_value={
                                    "title": "Stabilnejšie riadenie",
                                    "description": "Plynulejšie zákruty a opravy SDK.",
                                })):
            widget = UpdateCheckerWidget(object())
            widget.resize(240, 140)
            widget.show()
            self.app.processEvents()
            widget._on_checked(True, "abcdef0")

            self.assertEqual(widget.status_lbl.text(), "Dostupná nová verzia")
            self.assertNotIn("Stabilnejšie riadenie", widget.status_lbl.text())
            widget.btn.hovered.emit(True)
            self.app.processEvents()
            self.assertTrue(widget.changes_popover.isVisible())
            self.assertEqual(widget.changes_popover.release_title.text(),
                             "Stabilnejšie riadenie")
            self.assertEqual(widget.changes_popover.release_description.text(),
                             "Plynulejšie zákruty a opravy SDK.")
            button_bottom = widget.btn.mapToGlobal(
                widget.btn.rect().bottomLeft()).y()
            self.assertGreater(widget.changes_popover.y(), button_bottom)

            widget.btn.hovered.emit(False)
            self.app.processEvents()
            self.assertFalse(widget.changes_popover.isVisible())
            widget.close()

    def test_startup_shows_installation_before_initializing(self):
        splash = BootSplash()
        splash.show_update_installation(duration_ms=20)
        self.assertEqual(splash.status_lbl.text(),
                         "Inštalácia aktualizácie…")
        QTest.qWait(35)
        self.app.processEvents()
        self.assertEqual(splash.status_lbl.text(), "Initializing...")
        splash.close()


if __name__ == "__main__":
    unittest.main()
