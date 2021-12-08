from mock import patch

from analysis_report import get_summary, send_summary

class TestGetSummary:

    @patch('analysis_report.get_analysis_summary')
    def test_get__success(self, _g):
        _g.return_value = "Analysis summary"

        summary = get_summary()

        assert summary == 'Analysis summary'
        assert _g.called is True


    @patch('analysis_report.get_analysis_summary')
    def test_get__failure(self, _g):
        _g.side_effect = ValueError("Cannot do analysis")

        summary = get_summary()
        assert summary == 'Analysis cannot be retrieved! {}'.format("Cannot do analysis")
        assert _g.called is True


class TestSendSummary:

    @patch('os.system')
    def test_send(self, _s):
        send_summary('Analysis summary', 'Shenzhen, Guangdong, China')

        assert _s.called is True