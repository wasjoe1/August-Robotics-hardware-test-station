import sys
sys.path.append('../')
import os

import shutil
import pandas as pd
from datetime import timedelta
from mock import Mock, patch
import unittest

import dataparser
import extractor


class TestDataParser(unittest.TestCase):

    @patch('dataparser.DataParser.__init__')
    def test_init_dataparser(self, mock_init):
        """ Assert that a Dataparser object is created when called """

        mock_init.return_value = None
        d = dataparser.DataParser('testIn','testOut')
        mock_init.assert_called_with("testIn","testOut")
        assert isinstance(d, dataparser.DataParser)

    @patch('dataparser.PreprocessedFile.__init__')
    def test_init_preprocessed_success(self, mock_init):
        """ Assert that a Preprocessed object is created when called """
        
        mock_init.return_value = None
        d = dataparser.PreprocessedFile('testIn','testOut')
        mock_init.assert_called_with("testIn","testOut")
        assert isinstance(d, dataparser.PreprocessedFile)


    @patch('dataparser.FormattedFile.__init__')
    def test_init_formatted_success(self, mock_init):
        """ Assert that a FormattedFile object is created when called """
        
        mock_init.return_value = None
        d = dataparser.FormattedFile('outdir')
        mock_init.assert_called_with("outdir")
        assert isinstance(d, dataparser.FormattedFile)

    @patch('dataparser.ProcessedFile.__init__')
    def test_init_processed_success(self, mock_init):
        """ Assert that a ProcessedFile object is created when called """
        
        mock_init.return_value = None
        d = dataparser.ProcessedFile('filepath', 'analysis_type')
        mock_init.assert_called_with('filepath', 'analysis_type')
        assert isinstance(d, dataparser.ProcessedFile)

    @patch('extractor.generate_exception_csv', return_value=True)
    @patch('extractor.gather_useful', return_value=([],[]))
    @patch('extractor.generate_extracted_keywords', return_value='')
    @patch('extractor.pd.read_csv', return_value=pd.DataFrame([{"time": "", "msg": ""}]))
    def test_preprocess_data_file_ValueError(self, mock_read_csv, mock_generate_extracted_keywords, mock_gather_useful, mock_generate_exception_csv):
        """ 
        Assert that method raises a ValueError with invalid input
        """

        dp = dataparser.DataParser('','')
        with self.assertRaises(ValueError): 
            dp.preprocess_data_file()

    @patch('extractor.generate_exception_csv', return_value=True)    
    @patch('extractor.generate_extracted_keywords', return_value='')
    @patch('extractor.pd.read_csv', return_value=pd.read_csv(os.path.join(os.getcwd(), "fixtures/preprocess_data_file_success.csv"), float_precision='round_trip'))
    def test_preprocess_data_file_success(self, mock_read_csv, mock_generated_extracted_keywords, mock_generate_exception_csv):
        """
        Assert that the method returns a preprocessed file when it is
        successful
        """
        dp = dataparser.DataParser("","")
        preprocessed_file = dp.preprocess_data_file()
        assert isinstance(preprocessed_file, dataparser.PreprocessedFile)

    @patch('extractor.generate_exception_csv', return_value=("",None,None))
    @patch('extractor.remove_unwanted')
    @patch('extractor.generate_extracted_keywords', return_value='')
    @patch('extractor.pd.read_csv', return_value=pd.read_csv(os.path.join(os.getcwd(), "fixtures/preprocess_data_file_exception_present.csv"), float_precision='round_trip'))
    def test_preprocess_data_file_exception_present(self, mock_read_csv, mock_generated_extracted_keywords, mock_remove_unwanted, mock_generate_exception_csv):
        """
        Assert that the method returns a preprocessed file and accounts for the
        exceptions present
        """
        dp = dataparser.DataParser("","")
        preprocessed_file = dp.preprocess_data_file()

        # assert that remove exception from useful list is called
        assert mock_remove_unwanted.called
        assert isinstance(preprocessed_file, dataparser.PreprocessedFile)

    @patch('dataparser.isinstance', return_value=True)
    @patch('dataparser.LogFileFormatter.__init__', return_value=None)
    @patch('dataparser.LogFileFormatter.try_to_format', return_value=["",""])
    def test_format_data_file_success(self, mock_try_to_format, mock_init, mock_isinstance):
        """
        Assert that method will return a FormattedFile obj if a PreprocessedFile obj is passed
        """
        preprocessed_file = dataparser.PreprocessedFile('testIn','testOut')
        dp = dataparser.DataParser("","")
        assert isinstance(dp.format_data_file(preprocessed_file), dataparser.FormattedFile)
    
    @patch('dataparser.isinstance', return_value=False)
    def test_format_data_file_return_ValueError(self, mock_isinstance):
        """
        Assert that method retuns ValueErro when input is empty
        """
        dp = dataparser.DataParser('','')
        with self.assertRaises(ValueError): 
            dp.format_data_file("")        
 
    @patch('dataparser.isinstance', return_value=True)
    @patch('extractor.generate_statistics', return_value=(
        timedelta(microseconds=0),
        timedelta(microseconds=0),
        timedelta(microseconds=0),
        timedelta(microseconds=0),
        timedelta(microseconds=0)))
    def test_perform_analysis_success_dict_None(self, mock_generate_statistics, mock_isinstance):
        """
        Assert that method calls relevant method when dict is None
        """
        formatted_filepath = os.path.join(os.getcwd(), "fixtures/formatted_file.csv")        
        dp = dataparser.DataParser("", "")
        dp.exception_dict = None
        formatted_file = dataparser.FormattedFile(formatted_filepath)
        dp.num_list=[0,0,0, timedelta(microseconds=0)]
        dp.perform_analysis(formatted_file)
        assert mock_generate_statistics.called    

    @patch('dataparser.isinstance', return_value=True)
    @patch('extractor.generate_statistics', return_value=(
        timedelta(microseconds=0),
        timedelta(microseconds=0),
        timedelta(microseconds=0),
        timedelta(microseconds=0),
        timedelta(microseconds=0)))
    def test_perform_analysis_success_dict_Not_None(self, mock_generate_statistics, mock_isinstance):
        """
        Assert method calls relevant method when dict is not none
        """
        dp = dataparser.DataParser("", "")
        dp.exception_dict = ({
            "move": 0,
            "gbm": 0,
            "unknown": 0,
            "unknown_id": []
        })
        formatted_filepath = os.path.join(os.getcwd(), "fixtures/formatted_file.csv")
        formatted_file = dataparser.FormattedFile(formatted_filepath)   
        dp.num_list=[0,0,0, timedelta(microseconds=0)]
        dp.perform_analysis(formatted_file)
        assert mock_generate_statistics.called    

    def test_perform_analysis_ValueError(self):
        """
        Assert method raises Value Error if input file is NOT a FormattedFile
        """
        preprocessed_file = dataparser.PreprocessedFile('testIn','testOut')
        dp = dataparser.DataParser("", "")
        with self.assertRaises(ValueError):dp.perform_analysis(preprocessed_file)

    @patch('dataparser.timedelta', return_value=timedelta(seconds=0))
    @patch("extractor.apply_threshold_on_statistics", return_value=(
        timedelta(seconds=0),
        timedelta(seconds=0),
        timedelta(seconds=0),
        timedelta(seconds=0)))
    @patch('extractor.compare_two_stats', return_value = (0,0,0))
    def test_print_comparisons(self, mock_compare_two_stats, mock_apply_threshold_on_statistics, mock_timedelta):
        dp = dataparser.DataParser("", "")
        threshold_list = [1,2]
        dp.print_comparisons(threshold_list, "")

    def test_print_comparisons_threshold_empty(self):
        dp = dataparser.DataParser("", "")
        threshold_list = []
        with self.assertRaises(ValueError):dp.print_comparisons(threshold_list, "")
        
    def test_print_comparisons_threshold_zero(self):
        dp = dataparser.DataParser("", "")
        threshold_list = [0]
        with self.assertRaises(ValueError):dp.print_comparisons(threshold_list, "")
        
    def test_swap_call_back_statements_GA_GNG(self):
        """
        Assert that method will not swap Goal Accepted and Got new goal
        and returns the same df.
        """

        data = []
        #  Start, End
        data.append({
            'time': '2019-07-29 05:38:46,927',
            'msg': 'nav: Goal accepted, id: 2101, coords: (8.5, 27.0, 3.14159265358979)'
        })
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'gbm: Got new goal, internal id: 5, at: (3.9999999999999987, 3.365), rz: 1.57079632679'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        dp = dataparser.DataParser("", "")
        df_compare = df.copy(deep=True)
        df = dp.swap_call_back_statements(df)
        self.assertEqual(df_compare.iat[0,0], df.iat[0,0])

    def test_swap_call_back_statements_GNG_GA(self):
        """
        Assert that method will swap Goal Accepted and Got new goal
        and returns a diff df value.
        """
        data = []
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'gbm: Got new goal, internal id: 5, at: (3.9999999999999987, 3.365), rz: 1.57079632679'
        })

        data.append({
            'time': '2019-07-29 05:38:46,927',
            'msg': 'nav: Goal accepted, id: 2101, coords: (8.5, 27.0, 3.14159265358979)'
        })

        #  Csv will have at least 3 col
        data.append({
            'time': '',
            'msg': ''
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        dp = dataparser.DataParser("", "")
        df_compare = df.copy(deep=True)
        df = dp.swap_call_back_statements(df)
        #  Only the msg change place
        self.assertNotEqual(df_compare.iat[0,1], df.iat[0,1])

    def test_swap_call_back_statements_GNG_GA_GNG(self):
        """
        Assert that method will NOT swap Goal Accepted and Got new goal
        and returns a the last 2 elements in df value.
        """
        data = []
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'gbm: Got new goal, internal id: 5, at: (3.9999999999999987, 3.365), rz: 1.57079632679'
        })

        data.append({
            'time': '2019-07-29 05:38:46,927',
            'msg': 'nav: Goal accepted, id: 2101, coords: (8.5, 27.0, 3.14159265358979)'
        })

        #  Csv will have at least 3 col
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'gbm: Got new goal, internal id: 5, at: (3.9999999999999987, 3.365), rz: 1.57079632679'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        dp = dataparser.DataParser("", "")
        df_compare = df.copy(deep=True)
        df = dp.swap_call_back_statements(df)
        #  Only the msg change place
        self.assertEqual(df_compare.iat[0,1], df.iat[0,1])
        self.assertEqual(df_compare.iat[0,0], df.iat[0,0])
        self.assertEqual(df_compare.iat[1,1], df.iat[1,1])
        self.assertEqual(df_compare.iat[1,0], df.iat[1,0])
        self.assertEqual(df_compare.iat[2,1], df.iat[2,1])
        self.assertEqual(df_compare.iat[2,0], df.iat[2,0])

if __name__ == "__main__":
    unittest.main()
