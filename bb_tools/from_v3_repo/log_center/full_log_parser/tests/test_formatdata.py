import sys
sys.path.append('../')
import os

import pandas as pd
from datetime import timedelta
import datetime
import unittest
from mock import Mock, patch
import utils
import format_data
import dataparser

class TestFormatMethod(unittest.TestCase):
        
    def setup_method(self):
        pfile = dataparser.PreprocessedFile(os.path.join(os.getcwd(),"fixtures/preprocess_data_file_success.csv"),"some_filedir")
        with patch('format_data.pd.read_csv', return_value=True) as mock_to_csv, patch('format_data.len', return_value=0) as mock_len:
            lf = format_data.LogFileFormatter(pfile, 'some_outputdir')
            return lf

    @patch('format_data.LogFileFormatter.__init__')
    def test_LogFileFormatter_init_success(self, mock_init):
        """
        Assert that a LogFileFormatter obj is created
        """
        mock_init.return_value = None
        d = format_data.LogFileFormatter('','')
        mock_init.assert_called_with('','')
        assert isinstance(d, format_data.LogFileFormatter)

    def test_get_start_success(self):
        """
        Assert that get success will update the corresponding values
        """
        lf = self.setup_method()

        lf.get_start('some_line','date time')
        self.assertEqual(lf.depart_time, "time")
        self.assertEqual(lf.new_entry, False)
        self.assertEqual(lf.point, 1)
        self.assertEqual(len(lf.finaldata), 59)
        self.assertEqual(lf.finaldata[0], str(1))

    def test_get_tr_pos_self_movement_zero(self):
        lf = self.setup_method()

        line = "gbm: Got new goal, internal id: 2, at: (8.5, 2.6336), rz: -1.57079632679"
        lf.get_tr_pos(line)
        self.assertEqual(lf.finaldata[1], "8.5") #  TR_X
        self.assertEqual(lf.finaldata[2], "2.6336") #  TR_Y
        self.assertEqual(lf.finaldata[3], "-1.57079632679") #  TR_RZ
        self.assertEqual(lf.movement, 1)
        self.assertEqual(lf.first_entry, False)

    def test_get_tr_pos_self_movement_nonzero(self):
        lf = self.setup_method()

        line = "gbm: Got new goal, internal id: 2, at: (8.5, 2.6336), rz: -1.57079632679"
        lf.movement = 1
        lf.first_entry = False

        lf.get_tr_pos(line)
        self.assertEqual(lf.finaldata[10], "8.5") #  TR_X
        self.assertEqual(lf.finaldata[11], "2.6336") #  TR_Y
        self.assertEqual(lf.finaldata[12], "-1.57079632679") #  TR_RZ
        self.assertEqual(lf.movement, 2)

    @patch("format_data.re.split")
    def test_get_start_move_time(self, mock_re_split):
        lf = self.setup_method()
        date_time = "2019-08-20 12:17:46.291"
        line = "nav: move_base actived."
        lf.get_start_move_time(line, date_time)
        mock_re_split.assert_called_with(r'[,\s()]', line)

    def test_get_end_move_time(self):
        lf = self.setup_method()
        lf.move_start_time = "12:17:45.290"
        lf.move_end_time = "00:00:00.000"
        lf.movement = 1
        date_time = "2019-08-20 12:17:46.291"
        FMT = '%H:%M:%S.%f'
        tdelta = datetime.datetime.strptime(lf.move_end_time, FMT) - datetime.datetime.strptime(lf.move_start_time, FMT)
        line = "nav: Move base done to BoothBotNavigationState.MOVING"
        
        lf.get_end_move_time(line, date_time)
        #  Movement == 1
        self.assertEqual(lf.finaldata[utils.MOVETIME_COL], "0:00:01.001000")
        self.assertEqual(lf.time_checker, datetime.timedelta(0, 1, 1000))
        #  Movement > 1
        lf = self.setup_method()
        lf.move_start_time = "12:17:46.291"
        lf.move_end_time = "12:17:47.292"
        lf.movement = 2
        date_time = "2019-08-20 12:17:46.291"
        FMT = '%H:%M:%S.%f'
        tdelta = datetime.datetime.strptime(lf.move_end_time, FMT) - datetime.datetime.strptime(lf.move_start_time, FMT)
        line = "nav: Move base done to BoothBotNavigationState.MOVING"
        lf.get_end_move_time(line, date_time)
        #  The other movetime col should be 0
        self.assertEqual(lf.finaldata[16], "0:00:00")

    @patch("format_data.re.split")
    def test_get_localisation_start_time_regex(self, mock_re_split):
        """
        Assert that method calls the regex when start_local flag is true
        """
        lf = self.setup_method()
        date_time = "2019-08-20 12:17:46.291"
        line = "nav: GBM actived."
        lf.start_local = True
        lf.get_localisation_start_time(line, date_time)
        assert mock_re_split.called

    @patch("format_data.LogFileFormatter.record_intermediate_values")
    def test_get_localisation_start_time_start_find_local_end_true_one(self, mock_record_intermediate_values):
        """
        Assert that method calls the regex when start_local flag is true
        """
        lf = self.setup_method()
        date_time = "2019-08-20 12:17:46.291"
        line = "nav: GBM actived."
        lf.start_local = True
        lf.start_find_local_end = True
        lf.movement = 1 
        lf.get_localisation_start_time(line, date_time)
        mock_record_intermediate_values.called
    
    @patch("format_data.LogFileFormatter.perform_no_lcsn_movement_above_one")
    def test_get_localisation_start_time_start_find_local_end_true_above_one(self, mock_perform_no_lcsn_movement_above_one):        
        lf = self.setup_method()
        date_time = "2019-08-20 12:17:46.291"
        line = "nav: GBM actived."
        lf.start_local = False
        lf.start_find_local_end = True
        lf.movement = 3
        lf.get_localisation_start_time(line, date_time)
        assert mock_perform_no_lcsn_movement_above_one.called

    @patch("format_data.LogFileFormatter.record_intermediate_values")
    def test_get_localisation_start_time_start_find_local_end(self, mock_inter_values):
        """
        Assert that method calls the record_intermediate values if movement ==
        1 and start_find_local_end flag is true
        """
        lf = self.setup_method()
        date_time = "2019-08-20 12:17:46.291"
        line = "nav: GBM actived."
        lf.start_local = False
        lf.flipped_gbm = False
        lf.start_find_local_end = True
        lf.movement = 1
        lf.get_localisation_start_time(line, date_time)
        assert mock_inter_values.called

    @patch("format_data.LogFileFormatter.perform_no_lcsn_movement_above_one")
    def test_get_localisation_start_time_pfm_no_lcsn_movement(self, mock_lcsn_above_one):
        """
        Assert that method calls the perform_no_lcsn_movement_above_one method
        if movement >1 and start_find_local_end flag is true
        """
        lf = self.setup_method()
        date_time = "2019-08-20 12:17:46.291"
        line = "nav: GBM actived."
        lf.start_local = False
        lf.flipped_gbm = False
        lf.start_find_local_end = True
        lf.movement = 2
        lf.get_localisation_start_time(line, date_time)
        assert mock_lcsn_above_one.called

    @patch('math.hypot', return_value=0)
    def test_perform_no_lcsn_movement_above_one(self, mock_hypot):
        """
        Assert that after method called, the correct cells are updated
        accordingly
        """

        lf = self.setup_method()
        lf.local_x = 0
        lf.local_y = 0
        lf.local_rz = 0
        lf.tr_x = 0
        lf.tr_y = 0
        lf.tr_z = 0
        lf.local_end_time = "0:00:00.000"
        lf.local_start_time = "0:00:00.000"
        lf.start_find_local_end = True
        lf.movement = 2

        lf.perform_no_lcsn_movement_above_one()
        self.assertEqual(lf.finaldata[13], 0)
        self.assertEqual(lf.finaldata[14], 0)
        self.assertEqual(lf.finaldata[15], 0)
        self.assertEqual(lf.finaldata[18], 0)
        self.assertEqual(str(lf.time_checker), "0:00:00")
        self.assertEqual(str(lf.finaldata[17]), "0:00:00")
        self.assertEqual(lf.start_find_local_end, False)

 
    def test_record_intermediate_values(self):
        lf = self.setup_method()
        lf.local_x = 1
        lf.local_y = 2
        lf.local_rz = 3
        lf.tr_x = 0
        lf.tr_y = 0
        lf.local_end_time = "0:00:02.002"
        lf.local_start_time = "0:00:01.001"
        lf.record_intermediate_values()
        self.assertEqual(lf.finaldata[utils.AC_X], 1)
        self.assertEqual(lf.finaldata[utils.AC_Y], 2)
        self.assertEqual(lf.finaldata[utils.AC_RZ], 3)
        self.assertEqual(lf.finaldata[utils.ACC_COL], 2.23606797749979)
        self.assertEqual(lf.finaldata[utils.LCSN_COL], timedelta(0, 1, 1000))

    @patch("format_data.re.split")
    def test_get_localisation_end_time(self, mock_re_split):
        lf = self.setup_method()
        date_time = "2019-08-20 12:17:46.291"
        line = "gbm: Base estimated pose are ((9.469522442469811, 22.402111551837336), -1.6438182970172166)"
        lf.get_localisation_end_time(line,date_time)
        assert mock_re_split.called

    @patch('format_data.math.hypot', return_value=4)
    def test_get_final_pos(self, mock_hypot):
        """
        Assert that  method will update values in list accordingly
        """
        lf = self.setup_method()
        lf.get_final_pos_flag = False
        lf.start_find_local_end = True
        lf.local_x = 1
        lf.local_y = 2
        lf.local_rz = 3
        lf.tr_x = 0
        lf.tr_y = 0
        lf.tr_z = 0        
        lf.movement = 2
        lf.local_end_time = "0:00:00.000"
        lf.local_start_time = "0:00:00.000"
        lf.get_final_pos()
        self.assertEqual(lf.get_final_pos_flag, True)
        self.assertEqual(lf.start_find_local_end, False)
        self.assertEqual(lf.finaldata[13], 1)
        self.assertEqual(lf.finaldata[14], 2)
        self.assertEqual(lf.finaldata[15], 3)
        self.assertEqual(lf.finaldata[18], 4)

    @patch('format_data.math.hypot', return_value=4)
    def test_get_final_pos_ValueError(self, mock_hypot):
        """
        Assert that method will raise valueerror when movement <1
        """
        lf = self.setup_method()
        lf.get_final_pos_flag = False
        lf.start_find_local_end = True
        lf.local_x = 1
        lf.local_y = 2
        lf.local_rz = 3
        lf.tr_x = 0
        lf.tr_y = 0
        lf.tr_z = 0        
        lf.movement = 0
        lf.local_end_time = "0:00:00.000"
        lf.local_start_time = "0:00:00.000"
        with self.assertRaises(ValueError):lf.get_final_pos()

    def test_get_end_time(self):
        lf = self.setup_method()
        date_time = "2019-08-20 12:17:46.291"
        line = "BBC: goal 8601 finished"
        lf.marking_end_time = "0:00:00.000"
        lf.marking_start_time = "0:00:00.000"
        lf.end_time = "0:00:0.000"
        lf.depart_time = "0:00:0.000"    
        lf.local_start_time = "0:00:0.000"  
        lf.local_end_time = "0:00:0.000"  
        lf.movement = 2
        lf.get_final_pos_flag = True
        lf.dfinal = pd.DataFrame(index=range(0, utils.MAX_COL), columns=utils.table_col)
        lf.row_count = 1 
        
        lf.get_end_time(line, date_time)

        self.assertEqual(lf.finaldata[utils.MARK_COL], datetime.timedelta(0, 44266, 291000))
        self.assertEqual(lf.finaldata[utils.TIMECHECK_COL], datetime.timedelta(0, 44266, 291000))
        self.assertEqual(lf.time_checker, datetime.timedelta(0))
        #  Lcsn column for movement 2
        self.assertEqual(lf.finaldata[17], datetime.timedelta(0))
        self.assertEqual(lf.row_count, 2)
        self.assertEqual(lf.first_entry, True)
        self.assertEqual(lf.get_final_pos_flag, False)

    @patch('format_data.LogFileFormatter.format', side_effect=pd.errors.EmptyDataError)
    def test_try_to_format_empty_data_error(self, mock_format):
        lf = self.setup_method()
        lf.filepath = ""
        lf.output_dir = ""
        with self.assertRaises(ValueError):lf.try_to_format()

    @patch('format_data.LogFileFormatter.get_start')
    @patch('format_data.LogFileFormatter.get_tr_pos')
    @patch('format_data.LogFileFormatter.get_start_move_time')
    @patch('format_data.LogFileFormatter.get_end_move_time')
    @patch('format_data.LogFileFormatter.get_localisation_start_time')
    @patch('format_data.LogFileFormatter.get_localisation_end_time')
    @patch('format_data.LogFileFormatter.get_end_time')
    @patch('format_data.LogFileFormatter.get_final_pos')
    @patch('dataparser.tempfile.mkdtemp', return_value=True)
    @patch("format_data.pd.DataFrame.to_csv", return_value=True)
    @patch('os.path.join', return_value = "something")
    @patch("dataparser.DataParser.preprocess_data_file", return_value=dataparser.PreprocessedFile("", os.path.join(os.getcwd(),"fixtures/preprocess_data_file_success.csv")))
    @patch('extractor.generate_extracted_keywords', return_value='')
    @patch('extractor.pd.read_csv', return_value=pd.read_csv(os.path.join(os.getcwd(), "fixtures/preprocess_data_file_success.csv"), float_precision='round_trip'))
    def test_format_success(self, mock_read_csv, mock_generated_extracted_keywords, mock_preprocessed_file, mock_path_join, mock_to_csv,  mock_mkdtemp, mock_get_final_pos, mock_get_end_time, mock_get_lcsn_end_time, mock_get_lcsn_start_time, mock_get_end_move_time, mock_get_start_move_time, mock_get_tr_pos, mock_get_start):
        """
        Assert that method will access all methods and filter accordingly when
        passed in a valid preprocesed csv with data
        """
        lf = self.setup_method()
        filepath = os.path.join(os.getcwd(),"fixtures/preprocess_data_file_success.csv")
        lf.df = pd.read_csv(os.path.join(os.getcwd(), filepath),  index_col=None,float_precision='round_trip')
        lf.base_estimate_pose_present = True
        lf.dfinal = pd.DataFrame(index=range(0, utils.MAX_COL), columns=utils.table_col)
        lf.row_count = 1 
        
        dp = dataparser.DataParser("","")
        preprocessed_file = dp.preprocess_data_file()
        lf.format(preprocessed_file, "output_dir")
        assert mock_get_end_time.called
        assert mock_get_lcsn_end_time.called
        assert mock_get_lcsn_start_time.called
        assert mock_get_end_move_time.called
        assert mock_get_start_move_time.called
        assert mock_get_tr_pos.called
        assert mock_get_start.called

        lf.silent_flag = True
        lf.format(preprocessed_file, "output_dir")
        assert mock_get_final_pos.called


    @patch('dataparser.tempfile.mkdtemp', return_value=True)
    @patch("format_data.re.match")
    @patch("format_data.pd.DataFrame.to_csv", return_value=True)
    @patch('os.path.join', return_value = "something")
    @patch("dataparser.DataParser.preprocess_data_file", return_value=dataparser.PreprocessedFile)
    @patch('extractor.generate_extracted_keywords', return_value='')
    @patch('extractor.pd.read_csv', return_value=pd.read_csv(os.path.join(os.getcwd(), "fixtures/preprocess_data_file_success.csv"), float_precision='round_trip'))
    def test_format_success_base_estimate_not_present(self, mock_read_csv, mock_generated_extracted_keywords, mock_preprocessed_file, mock_path_join, mock_to_csv, mock_re_match, mock_mkdtemp):

        lf = self.setup_method()
        filepath = os.path.join(os.getcwd(),"fixtures/preprocess_data_file_success.csv")
        lf.df = pd.read_csv(os.path.join(os.getcwd(), filepath),  index_col=None,float_precision='round_trip')
        lf.base_estimate_pose_present = False
        lf.dfinal = pd.DataFrame(index=range(0, utils.MAX_COL), columns=utils.table_col)
        lf.row_count = 1 
        
        dp = dataparser.DataParser("","")
        preprocessed_file = dp.preprocess_data_file()

        with self.assertRaises(ValueError):lf.format(preprocessed_file, "output_dir")



if __name__ == "__main__":
    unittest.main()
