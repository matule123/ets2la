from core.navigation import map_data


def test_missing_map_index_dependencies_keep_fallback_and_exact_error(monkeypatch):
    monkeypatch.setattr(map_data, "requests", None)
    monkeypatch.setattr(map_data, "yaml", None)
    monkeypatch.setattr(map_data, "_index_cache", None)
    result = map_data.get_index(force=True)
    assert set(map_data.COMPATIBILITY_DATASETS).issubset(result)
    assert "requests" in map_data.last_error()
    assert "PyYAML" in map_data.last_error()


def test_map_index_http_failure_preserves_status_url_and_fallback(monkeypatch):
    class Response:
        status_code = 503

    class Requests:
        @staticmethod
        def get(*_args, **_kwargs):
            return Response()

    class Yaml:
        pass

    monkeypatch.setattr(map_data, "requests", Requests)
    monkeypatch.setattr(map_data, "yaml", Yaml)
    monkeypatch.setattr(map_data, "_index_cache", None)
    result = map_data.get_index(force=True)
    assert result
    assert "HTTP 503" in map_data.last_error()
    assert map_data.INDEX_URL in map_data.last_error()
