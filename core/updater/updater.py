from textual.app import App, ComposeResult
from textual.widgets import Header, Log, Label, Static, Button
import asyncio
import time
import subprocess
import os
import sys

# Define the update steps
# In a real scenario, these would be tailored to the actual repo structure
STEPS = [
    {"name": "Checking for Updates", "command": "git fetch"},
    {"name": "Applying Updates", "command": "git pull"},
    {"name": "Updating Dependencies", "command": "pip install -r requirements.txt"},
    {"name": "Verifying Assets", "command": "python -c \"print('Assets verified')\""},
]

class UltraPilotUpdater(App):
    """Professional Updater for ETS2-UltraPilot."""

    CSS_PATH = "updater.tcss"

    def on_mount(self) -> None:
        self.title = "ETS2-UltraPilot Updater"
        self.sub_title = "Bringing your autopilot to the next level..."

    async def on_ready(self) -> None:
        self.icon = "◐"
        self.frame = 0
        self.set_interval(0.25, self.update_icon)
        time.sleep(1)
        await self.run_steps()

    def compose(self) -> ComposeResult:
        yield Header(show_clock=True)

        sidebar = Static(classes="sidebar")
        with sidebar:
            yield Label("--  Update Progress  --")
            for step in STEPS:
                yield Label(f"○ {step['name']}", classes="not-done")
            yield Label("")
            yield Button("Retry", id="retry-button", disabled=True)
            yield Button("Exit", id="exit-button", disabled=True)

        yield sidebar

        log_container = Static(classes="box")
        with log_container:
            yield Log(auto_scroll=True, highlight=True, classes="log")

        yield log_container

    def update_icon(self):
        spinner = ["◐", "◓", "◑", "◒"]
        self.icon = spinner[self.frame % 4]
        self.frame += 1
        self.query_one(Header).icon = self.icon

    async def run_steps(self):
        log_widget = self.query_one(Log)
        sidebar = self.query_one(Static)

        for idx, step in enumerate(STEPS):
            label = sidebar.children[idx + 1]
            label.classes = ["doing"]

            log_widget.write(f"[bold blue]-- RUNNING {step['name']} --[/bold blue]\n")

            try:
                process = await asyncio.create_subprocess_shell(
                    step["command"],
                    stdout=asyncio.subprocess.PIPE,
                    stderr=asyncio.subprocess.PIPE,
                    shell=True,
                )

                async def read_stream(stream):
                    while True:
                        line = await stream.readline()
                        if line:
                            log_widget.write(line.decode().strip() + "\n")
                        else:
                            break

                await asyncio.gather(
                    read_stream(process.stdout),
                    read_stream(process.stderr),
                )

                await process.wait()

                if process.returncode != 0:
                    log_widget.write(f"[bold red]-- ERROR in {step['name']} --[/bold red]\n")
                    label.classes = ["error"]
                    label.update(f"X {step['name']}")

                    self.query_one("#retry-button", Button).disabled = False
                    self.query_one("#exit-button", Button).disabled = False
                    self._paused = True
                    return
                else:
                    log_widget.write(f"[bold green]-- COMPLETED {step['name']} --[/bold green]\n\n")
                    label.classes = ["done"]
                    label.update(f"● {step['name']}")

            except Exception as e:
                log_widget.write(f"[bold red]Exception occurred: {str(e)}[/bold red]\n")
                label.classes = ["error"]
                self._paused = True
                return

        if not getattr(self, "_paused", False):
            log_widget.write("\n[bold green]UltraPilot is now up to date! Restarting...[/bold green]\n")
            self.icon = "✔"
            self.refresh()
            time.sleep(3)
            self.exit()

    async def on_button_pressed(self, event) -> None:
        button = event.button
        if button.id == "retry-button":
            button.disabled = True
            self.query_one("#exit-button", Button).disabled = True
            await self.run_steps()
        elif button.id == "exit-button":
            self.exit()

if __name__ == "__main__":
    app = UltraPilotUpdater()
    app.run()
