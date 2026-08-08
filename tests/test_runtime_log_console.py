import logging

from core import logger


def test_console_line_matches_ets2la_tag_time_and_message_spacing():
    record = logging.LogRecord("Engine", logging.INFO, "engine.py", 42,
                               "Engine started", (), None)
    output = logger._ETS2LAFormatter().format(record)
    assert "\033[92m[INF]\033[0m " in output
    assert "  \033[97mEngine started\033[0m" in output
    assert "engine.py:42" not in output


def test_plugin_summary_uses_only_identified_plugin_messages():
    lines = [
        "2026-08-08 12:00:00,000 INFO     PluginManager Loaded plugin: autopilot",
        "2026-08-08 12:00:01,000 WARNING  autopilot Camera confidence low",
        "2026-08-08 12:00:02,000 ERROR    Engine [plugin:map] worker crashed",
        "2026-08-08 12:00:03,000 CRITICAL Engine Plugin 'map' restart failed",
        "2026-08-08 12:00:04,000 WARNING  Engine unrelated runtime warning",
    ]

    issues = logger.collect_plugin_issues(lines=lines)

    assert issues["autopilot"]["warnings"] == 1
    assert issues["map"]["errors"] == 2
    assert "engine" not in issues


def test_plugin_summary_has_separate_warning_and_error_frames():
    issues = {
        "autopilot": {"warnings": 2, "errors": 0,
                      "warning_messages": ["confidence low"],
                      "error_messages": []},
        "map": {"warnings": 0, "errors": 1,
                "warning_messages": [], "error_messages": ["worker crashed"]},
    }

    output = logger.format_plugin_issue_summary(issues, colour=False)

    assert "autopilot" in output
    assert "map" in output
    assert "Errors: 1" in output
    assert "Warnings: 2" in output
    assert "PLUGINY S UPOZORNENÍM" not in output


def test_runtime_log_waits_for_enter_only_when_plugin_has_issue(monkeypatch):
    prompts = []
    monkeypatch.setattr(logger, "collect_plugin_issues", lambda _offset: {
        "map": {"warnings": 1, "errors": 0,
                "warning_messages": ["missing optional field"],
                "error_messages": []},
    })

    logger.finish_session_log(10, input_fn=lambda prompt: prompts.append(prompt),
                              colour=False)
    assert len(prompts) == 1
    assert "Enter" in prompts[0]

    prompts.clear()
    monkeypatch.setattr(logger, "collect_plugin_issues", lambda _offset: {})
    logger.finish_session_log(10, input_fn=lambda prompt: prompts.append(prompt),
                              colour=False)
    assert prompts == []


def test_startup_banner_is_not_a_regular_timestamped_log_line(capsys):
    logger.print_startup_banner("0.4.2", "abcdef123")
    output = capsys.readouterr().out
    assert "UltraPilot  v0.4.2  ·  abcdef1" in output
    assert "Pripravujem bezpečné jazdné systémy" in output
    assert "INFO" not in output
