import multiprocessing as mp

# freeze_support() MUST run before anything else (especially before any other
# imports or sys.path edits).  In a frozen Windows build, every child process
# re-launches this executable; if freeze_support isn't the very first thing the
# interpreter does, the spawn handshake (handle duplication) can fail with
# "WinError 5: Access is denied".
if __name__ == "__main__":
    mp.freeze_support()

    import sys
    import os

    # When frozen, the exe dir is the base; from source, this file's folder is.
    if getattr(sys, "frozen", False):
        base = os.path.dirname(sys.executable)
    else:
        base = os.path.dirname(os.path.abspath(__file__))
    if base not in sys.path:
        sys.path.insert(0, base)

    # NOTE: the old blocking update-splash window used to run here. Updates are
    # now checked from inside the running app (sidebar update widget), so the
    # app starts straight away without a pre-launch delay.
    import bootloader
    bootloader.main()
