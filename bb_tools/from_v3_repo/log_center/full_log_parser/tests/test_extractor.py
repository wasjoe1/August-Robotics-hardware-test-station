import sys
import os
import StringIO
sys.path.append('../')
import shutil

import unittest
from datetime import timedelta, datetime
import collections
import pandas as pd
from mock import Mock, patch

import utils
import extractor


class TestExtractorMethod(unittest.TestCase):

    @patch('extractor.os.path.join', return_value=True)
    @patch('extractor.open')
    @patch('extractor.convert_to_csv')
    @patch('extractor.list', return_value=["placeholder"])
    def test_generate_extracted_keywords_success(self, mock_list, mock_convert_to_csv, mock_open, mock_os_path_join):
        """ Assert that the method will call convert_to_csv when success """

        test_file = extractor.generate_extracted_keywords("filedir", "test_outputdir", "tmpdir")
        mock_convert_to_csv.called


    @patch('extractor.list', return_value=[])
    def test_generate_extracted_keywords_directory_empty(self, mock_list):
        """
        Assert that the method will raise an error when the directory is empty
        """
        with self.assertRaises(ValueError):extractor.generate_extracted_keywords("", "", "")

    @patch('extractor.pd.DataFrame.to_csv', return_value=True)
    def test_convert_to_csv(self, mock_to_csv):
        filename = os.path.join(os.getcwd(),'fixtures/test1.log')
        path = extractor.convert_to_csv(filename, "output_dir", "")
        self.assertEqual(path, "sorted_extracted_log.csv")

    @patch('os.path.exists', return_value=False)
    def test_convert_to_csv_input_dir_invalid_ValueError(self, mock_os_path_exists):
        with self.assertRaises(ValueError):extractor.convert_to_csv("","","")

    def test_line_data_success(self):
        """ Assert that method will return two var when it is successful """

        p = "[rosout][INFO] 2019-08-14 05:48:31,806: nav: GBM actived."

        time, msg = extractor.line_data(p)

        self.assertEqual("2019-08-14 05:48:31.806000", str(time))
        self.assertEqual("nav: GBM actived.", str(msg))

    @patch('extractor.re.match', return_value=False)
    def test_line_data_ValueError(self, mock_re_match):
        """ Assert that method will return two var when it is successful """

        p = "[rosout][INFO] 2019-08-14 05:48:31,806: nav: GBM actived."

        with self.assertRaises(ValueError):extractor.line_data(p)

    @patch('extractor.re.match', return_value=None)   
    def test_line_data_fail(self, mock_re_match):
        """ Assert that method will raise and error if content is incorrect """
        with self.assertRaises(ValueError):extractor.line_data("")

class TestGetMethods(unittest.TestCase):

    def test_gather_useful_empty_success(self):
        """ Assert that method returns success two list when list is empty """
        
        data = [{
            "time": '',
            "msg": ''
        }]
        list_1, list_2 = extractor.gather_useful(pd.DataFrame(data))
        self.assertEqual(list_1, [])
        self.assertEqual(list_2, [0,0,0,0])

    def test_gather_useful_valid_start_end_mark_success(self):
        """ Assert that method returns success when only have valid start and end and valid marking time"""

        data = []
        #  Start, Mark, End
        data.append({
            'time': '2019-07-29 05:38:46,927',
            'msg': 'nav: Goal accepted, id: 2101, coords: (8.5, 27.0, 3.14159265358979)'
        })
        data.append({
            "time": '2019-07-29 05:39:00,927',
            "msg": 'BBC: Marking...'
        })
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'BBC: goal 2101 finished'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        list_1, list_2 = extractor.gather_useful(df)

        self.assertEqual(list_1, [("05:38:46,927","05:39:20,727", "2019-07-29", "2019-07-29")])
        self.assertEqual(list_2, [0,1,0,timedelta(0)])

    def test_gather_useful_valid_start_end_only(self):
        """
        Assert that the method will return an empty list if only start and end are present
        """
        data = []
        #  Start, End
        data.append({
            'time': '2019-07-29 05:38:46,927',
            'msg': 'nav: Goal accepted, id: 2101, coords: (8.5, 27.0, 3.14159265358979)'
        })
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'BBC: goal 2101 finished'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        list_1, list_2 = extractor.gather_useful(df)
        self.assertEqual(list_1, [])
        self.assertEqual(list_2, [0,0,1,timedelta(0)]) #  <fail, success, fail to mark>
        
    def test_gather_useful_start_only(self):
        """
        Assert that the method will return an empty list if only start is present
        """
        
        data = []
        #  Start
        data.append({
            'time': '2019-07-29 05:38:46,927',
            'msg': 'nav: Goal accepted, id: 2101, coords: (8.5, 27.0, 3.14159265358979)'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        list_1, list_2 = extractor.gather_useful(df)
        self.assertEqual(list_1, [])
        self.assertEqual(list_2, [0,0,0,timedelta(0)]) #  <fail, success, fail to mark>

    def test_gather_useful_end_only(self):
        """
        Assert that method returns empty list when only end is present
        """

        data = []
        #  End
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'BBC: goal 2101 finished'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        list_1, list_2 = extractor.gather_useful(df)
        self.assertEqual(list_1, [])
        self.assertEqual(list_2, [1,0,0,0])

    def test_gather_useful_mark_only(self):
        """
        Assert that method empty list when only mark is present
        """

        data = []
        #  Mark
        data.append({
            "time": '2019-07-29 05:39:00,927',
            "msg": 'BBC: Marking...'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        list_1, list_2 = extractor.gather_useful(df)
        self.assertEqual(list_1, [])
        self.assertEqual(list_2, [0,0,0,timedelta(0)])

    def test_gather_useful_start_mark_only(self):
        """
        Assert that method returns an empty list when only start and mark is present
        """

        data = []
        #  Start, Mark
        data.append({
            'time': '2019-07-29 05:38:46,927',
            'msg': 'nav: Goal accepted, id: 2101, coords: (8.5, 27.0, 3.14159265358979)'
        })
        data.append({
            "time": '2019-07-29 05:39:00,927',
            "msg": 'BBC: Marking...'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])

        df.sort_index(inplace=True)
        list_1, list_2 = extractor.gather_useful(df)
        self.assertEqual(list_1, [])
        self.assertEqual(list_2, [0,0,0,timedelta(0)])

    def test_gather_useful_mark_end_only(self):
        """
        Assert that method returns empty list when only mark and end present
        """

        data = []
        #  Mark, End
        data.append({
            "time": '2019-07-29 05:39:00,927',
            "msg": 'BBC: Marking...'
        })
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'BBC: goal 2101 finished'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        list_1, list_2 = extractor.gather_useful(df)
        self.assertEqual(list_1, [])
        self.assertEqual(list_2, [0,0,0,timedelta(0)])
    
    @patch("extractor.pd.DataFrame.to_csv", return_value = True)
    def test_remove_unwanted_filter_success(self, mock_to_csv):
        """
        Assert that the method will return a csv file when it has filtered some valid time ranges
        """
        
        #  Setup Dataframe
        useful_time_list = [
            ("13:24:27.938", "13:24:31.415",
            "2019-07-29", "2019-07-29")        
        ]
        my_data = []
        #  Contains 3 times, only 1 within range
        #  Before range
        my_data.append({
            "time": '2019-07-29 13:24:27.937',
            "msg": 'BBC: Marking...'
        })
        #  Within Range
        my_data.append({
            "time": '2019-07-29 13:24:29.938',
            "msg": 'BBC: Marking...'
        })

        #  Beyond Range
        my_data.append({
            "time": '2019-07-29 13:24:31.416',
            "msg": 'BBC: Marking...'
        })

        df = pd.DataFrame(my_data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        path = extractor.remove_unwanted(df, "output_dir", useful_time_list, "tmpdir") 
        mock_to_csv.assert_called_with("tmpdir/preprocessed.csv", header=True, float_format='%g')
        
        self.assertEqual(path, "tmpdir/preprocessed.csv")
        # Check if file exists
        # self.assertNotEqual(filepath, None)

    def test_remove_unwanted_None_raise_ValueError(self):
        """
        Assert that the method will raise a ValueError if there is no useful time range
        """
        
        #  Setup Dataframe
        useful_time_list = [
            ("13:24:27.938", "13:24:31.415",
            "2019-07-29", "2019-07-29")        
        ]
        my_data = []
        #  Contains only out of range
        #  Before range
        my_data.append({
            "time": '2019-07-29 13:24:27.937',
            "msg": 'BBC: Marking...'
        })

        #  Beyond Range
        my_data.append({
            "time": '2019-07-29 13:24:31.416',
            "msg": 'BBC: Marking...'
        })

        df = pd.DataFrame(my_data, columns=['time', 'msg'])
        df.sort_index(inplace=True)
        
        with self.assertRaises(ValueError): extractor.remove_unwanted(df, "output_dir", useful_time_list, "tmpdir") 

    def test_identify_exceptions_no_exceptions(self):
        """
        Assert that method will return a list of exceptions with a correct
        start sequence and correct order
        """
        #  Setup
        data = []
        #  Correct order Start>GBM
        data.append({
            "time": '2019-07-29 05:39:00,927',
            "msg": 'nav: Goal accepted, id: 18401, coords: (45.5, 85.47, -1.5707963267948966)'
        })
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'nav: GBM actived.'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)

        exception_time_list = extractor.identify_exceptions(df)
        self.assertEqual([], exception_time_list)
        
    def test_identify_exceptions_no_start_point_found_ValueError(self):
        """ Assert that method will return a list of exceptions if present """
        #  Setup        
        data = []
        #  Wrong order Marked>Goal Accepted
        data.append({
            "time": '2019-07-29 05:39:00,927',
            "msg": 'BBC: Marking...'
        })
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'gbm: Base estimated pose are ((39.86543143999395, 85.46800266436884), 0.006245617618419352)'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)

        with self.assertRaises(ValueError):extractor.identify_exceptions(df)

    def test_identify_exceptions_start_point_but_order_wrong(self):
        """
        Assert that method will raise a ValueError with a correct
        start sequence but wrong order
        """
        #  Setup
        data = []
        #  Correct order Start>move>move
        data.append({
            "time": '2019-07-29 05:39:00,927',
            "msg": 'nav: Goal accepted, id: 18401, coords: (45.5, 85.47, -1.5707963267948966)'
        })
        data.append({
            "time": '2019-07-29 05:39:20,727',
            "msg": 'nav: move_base actived.'
        })
        data.append({
            "time": '2019-07-29 05:40:20,727',
            "msg": 'nav: move_base actived.'
        })

        df = pd.DataFrame(data, columns=['time', 'msg'])
        df.sort_index(inplace=True)

        exception_time_list = extractor.identify_exceptions(df)
        self.assertEqual(1, len(exception_time_list))

    def test_identify_exceptions_success(self):
        """
        Assert that method will return a list of exceptions with a correct
        start sequence and correct order and the exceptions
        """
        df = pd.read_csv(os.path.join(os.getcwd(), "fixtures/preprocess_data_file_exception_present.csv"), float_precision='round_trip')

        exception_time_list = extractor.identify_exceptions(df)
        self.assertEqual(["2019-08-14 11:21:30.471"], exception_time_list)

    def test_remove_exception_from_useful_list_success(self):
        """
        Assert that if a value is present in both exception list and useful
        time list, it will be removed from useful time list and added its start
        range to a new exception list
        """
        #  Setup Dataframe
        useful_time_list = [
            ("13:24:27.938", "13:24:31.415",
            "2019-07-29", "2019-07-29")       
        ]
        #  Within Range
        exception_list = [
            '2019-07-29 13:24:29.938'
        ]
        resultant_useful_list, resultant_exception_list = extractor.remove_exception_from_useful_list(useful_time_list, exception_list)
        
        self.assertEqual([], resultant_useful_list)
        self.assertEqual([("13:24:27.938", "13:24:31.415",
            "2019-07-29", "2019-07-29")], resultant_exception_list)

    def test_remove_exception_from_useful_list_no_exception(self):
        """
        Assert that if a value is present in both exception list and useful
        time list, it will be removed from useful time list and added its start
        range to a new exception list
        """
        #  Setup Dataframe
        useful_time_list = [
            ("13:24:27.938", "13:24:31.415",
            "2019-07-29", "2019-07-29")       
        ]
        #  Outside of Range
        exception_list = [
            '2019-07-29 13:24:27.937'
        ]
        resultant_useful_list, resultant_exception_list = extractor.remove_exception_from_useful_list(useful_time_list, exception_list)
        
        self.assertEqual([("13:24:27.938", "13:24:31.415",
            "2019-07-29", "2019-07-29")], resultant_useful_list)
        self.assertEqual([], resultant_exception_list)
    
    def test_remove_exception_from_useful_list_empty_list(self):
        """
        Assert that useful_time_list is empty, it will throw a ValueError
        """
        useful_time_list = []
        exception_list = []   
        with self.assertRaises(AssertionError):extractor.remove_exception_from_useful_list(useful_time_list, exception_list)
    
    def test_create_exceptions_list_for_csv_success(self):
        my_data = []
        my_data.append({
            "time": '2019-07-29 13:20:27.937',
            "msg": 'Test1'
        })
        my_data.append({
            "time": '2019-07-29 13:25:29.938',
            "msg": 'This is an exception'
        })
        my_data.append({
            "time": '2019-07-29 13:30:31.416',
            "msg": 'Test3'
        })
        df = pd.DataFrame(my_data, columns=['time', 'msg'])
        df.sort_index(inplace=True)

        #  Contains the time range for that point with an exception
        exception_list = [("13:25:29.938", "13:25:29.938", "2019-07-29","2019-07-29")]
        resultant_data = extractor.create_exceptions_list_for_csv(df, exception_list)
        check_data = []
        check_data.append({
            "time": '2019-07-29 13:25:29.938',
            "msg": 'This is an exception'
        })
        check_data.append({
            "time": '',
            "msg": ''
        })
        self.assertEqual(check_data, resultant_data)
    
    def test_create_exceptions_list_for_csv_ValueError(self):
        """
        Assert that method raises a ValueError when resultant list is empty
        """
        my_data = []
        my_data.append({
            "time": '2019-07-29 13:20:27.937',
            "msg": 'Does not fall in exception range1'
        })
        my_data.append({
            "time": '2019-07-29 13:20:29.938',
            "msg": 'Does not fall in the exception range2'
        })

        df = pd.DataFrame(my_data, columns=['time', 'msg'])
        df.sort_index(inplace=True)

        #  Contains the time range for that point with an exception
        exception_list = [("13:25:29.938", "13:25:29.938", "2019-07-29","2019-07-29")]
        with self.assertRaises(ValueError):extractor.create_exceptions_list_for_csv(df, exception_list)
        
    def test_create_exceptions_list_for_csv_empty_list_ValueError(self):
        """
        Assert that method raises a ValueError when resultant list is empty
        """
        df = pd.DataFrame([])
        df.sort_index(inplace=True)

        #  Contains the time range for that point with an exception
        exception_list = []
        with self.assertRaises(ValueError):extractor.create_exceptions_list_for_csv(df, exception_list)        


    @patch('extractor.analyze_exception_list', return_value=([], timedelta(microseconds=0)))
    @patch('extractor.pd.DataFrame.to_csv', return_value=True)  
    @patch('extractor.create_exceptions_list_for_csv', return_value=[{"time":'',"msg":''}])
    def test_generate_exceptions_csv_success(self, mock_create_exceptions_list_for_csv,mock_to_csv, mock_analyze_exceptions_list):
        """
        Assert that method calls to_csv when no exceptions
        """
        extractor.generate_exception_csv(pd.DataFrame(), "output_dir", [])
        
        mock_to_csv.assert_called_with("output_dir/exceptions.csv", header=True, float_format='%g')
        

    def test_order_checker_all_keyword_success(self):
        """
        Asserts that the given keyword will output the its subsequent expected
        keywords as per in utils -> expected_regex
        """
        # for loop to loop all, use the same utils
        msg_list = [
            "nav: Goal accepted, id: 1501, coords: (8.5, 3.0, -1.5707963267948966)",
            "nav: GBM actived.",
            "gbm: Got new goal, internal id: 2, at: (8.5, 2.6336), rz: -1.57079632679",
            "nav: move_base actived.",
            "nav: Move base done to BoothBotNavigationState.MOVING",
            "nav: Localization takes 32.792932987s",
            "gbm: Base estimated pose are ((7.922601350076445, 2.7598306677024067), -1.638723982254973)",
            "BBC: Marking...",
            "BBC: goal 1201 finished"
        ]
        # hardcoded testing for now
        test_order = [
            'keyword_start',
            'keyword_start_localisation_no_goal',
            'keyword_tr_pos',
            'keyword_move_base_start',
            'keyword_move_base_end',
            'keyword_last_localisation',
            'keyword_end_localisation',
            'keyword_marked',
            'keyword_finish'
        ]

        i=0
        for line in msg_list:
            error_flag, next_word, timestamp, got_new_goal_flag = extractor.order_checker("", line, "first_entry", False, False) 
            self.assertEqual(error_flag, False)
            self.assertEqual(next_word, (utils.expected_regex[test_order[i]]))
            i += 1

    def test_order_checker_GBM_flipped_success(self):
        """
        Asserts that the given keyword will output the its subsequent expected
        keywords as per in utils -> expected_regex
        """
        # for loop to loop all, use the same utils
        msg_list = [
            "nav: Goal accepted, id: 1501, coords: (8.5, 3.0, -1.5707963267948966)",
            "gbm: Got new goal, internal id: 2, at: (8.5, 2.6336), rz: -1.57079632679",
            "nav: GBM actived.",
            "nav: move_base actived.",
            "nav: Move base done to BoothBotNavigationState.MOVING",
            "nav: Localization takes 32.792932987s",
            "gbm: Base estimated pose are ((7.922601350076445, 2.7598306677024067), -1.638723982254973)",
            "BBC: Marking...",
            "BBC: goal 1201 finished"
        ]
        
        test_order = [
            'keyword_start',
            'keyword_tr_pos',
            'keyword_start_localisation_no_goal',
            'keyword_move_base_start',
            'keyword_move_base_end',
            'keyword_last_localisation',
            'keyword_end_localisation',
            'keyword_marked',
            'keyword_finish'
        ]

        i=0
        for line in msg_list:
            error_flag, next_word, timestamp, got_new_goal_flag = extractor.order_checker("", line, "first_entry", False, False) 
            self.assertEqual(error_flag, False)
            self.assertEqual(next_word, (utils.expected_regex[test_order[i]]))
            i+=1

    def test_order_checker_all_combi(self):
        """
        Assert that when the cmds are called in any order, it will behave as
        expected only as in utils.py
        """
        # for loop to loop all, use the same utils
        test_order = {
            'keyword_start': utils.keyword_start,
            'keyword_start_localisation': utils.keyword_start_localisation,
            'keyword_tr_pos': utils.keyword_tr_pos,
            'keyword_move_base_start' : utils.keyword_move_base_start,
            'keyword_move_base_end' : utils.keyword_move_base_end,
            'keyword_last_localisation' : utils.keyword_last_localisation,
            'keyword_end_localisation': utils.keyword_end_localisation,
            'keyword_marked' : utils.keyword_marked,
            'keyword_finish' : utils.keyword_finish

        }

        msg_list = [
            "nav: Goal accepted, id: 1501, coords: (8.5, 3.0, -1.5707963267948966)",
            "nav: GBM actived.",
            "gbm: Got new goal, internal id: 2, at: (8.5, 2.6336), rz: -1.57079632679",
            "nav: move_base actived.",
            "nav: Move base done to BoothBotNavigationState.MOVING",
            "nav: Localization takes 32.792932987s",
            "gbm: Base estimated pose are ((7.922601350076445, 2.7598306677024067), -1.638723982254973)",
            "BBC: Marking...",
            "BBC: goal 1201 finished",
        ]        

        true_false_list = []
        for x in range(len(test_order)):
            #  First pass in a line and desired next word
            test_error_flag, test_next_word, test_timestamp, got_new_goal_flag = extractor.order_checker('', msg_list[x], "first_entry", False, False)
            #  Then loop the result
            for i in range(len(test_order)):
                #  Ignore the ones to itself, only focus on others
                if i == x :
                    continue
                error_flag, next_word, timestamp, got_new_goal_flag = extractor.order_checker('', msg_list[i], test_next_word, False, False)
                status = False
                if error_flag is True:
                    if test_order.keys()[i] in test_next_word:
                        status = True
                    self.assertEqual(False , status)
                elif error_flag is False:
                    if (test_order.keys()[i] not in test_next_word) :
                        status = True
                    self.assertEqual(True, status)

    def test_analyze_exception_list_success(self):
        """
        Assert that method will return correct total duration value and
        breakdown list with fixed exception list
        """
        df = pd.read_csv(os.path.join(os.getcwd(), "fixtures/exception_list.csv"),  index_col=None,float_precision='round_trip')
        #print(df)
        df = df.drop(df.columns[0], axis=1)
        # df drop first col        
        breakdown_list, total_duration = extractor.analyze_exception_list(df)
        self.assertEqual(breakdown_list, {'unknown': 2, 'gbm': 0, 'unknown_id': ['8701', '24701']})
        self.assertEqual(total_duration, timedelta(0))

class TestStatisticalCalculation(unittest.TestCase):

    def test_calculate_total_localization_time(self):
        df = pd.read_csv(os.path.join(os.getcwd(), "fixtures/formatted_file.csv"),  index_col=None,float_precision='round_trip')    
        time = extractor.calculate_total_localization_time(df)
        self.assertEqual("0:04:25.368000", str(time))
          
    def test_calculate_total_movement_time(self):
        df = pd.read_csv(os.path.join(os.getcwd(), "fixtures/formatted_file.csv"),  index_col=None,float_precision='round_trip')    
        time = extractor.calculate_total_movement_time(df)
        self.assertEqual("0:02:25.040000", str(time))

    def test_calculate_total_duration(self):
        df = pd.read_csv(os.path.join(os.getcwd(), "fixtures/formatted_file.csv"),  index_col=None,float_precision='round_trip')    
        time = extractor.calculate_total_duration(df, 56)
        #  Using total marking time
        self.assertEqual("0:00:29.927000", str(time))

    def test_filter_threshold(self):
        df = pd.read_csv(os.path.join(os.getcwd(), "fixtures/formatted_file.csv"),  index_col=None,float_precision='round_trip') 
        df_copy = df.copy(deep=True)
        extractor.filter_threshold(df, 0.005)
        for i in range(len(df)):
            for j in range(12):
                if (df.iat[j,i])!=(df.iat[j,i]):
                    continue
                self.assertEqual(df.iat[j,i], df_copy.iat[j,i])

    @patch('extractor.calculate_total_duration',return_value=0)
    @patch('extractor.calculate_total_movement_time',return_value=0)
    @patch('extractor.calculate_total_localization_time', return_value=0)
    def test_generate_statistics(self, mock_lcsn_time, mock_movement_time, mock_duration):
        
        extractor.generate_statistics(os.path.join(os.getcwd(),"fixtures/formatted_file.csv"))
        assert mock_lcsn_time.called
        assert mock_movement_time.called
        assert mock_duration.called

    @patch('extractor.calculate_total_duration',return_value=0)
    @patch('extractor.calculate_total_movement_time',return_value=0)
    @patch('extractor.calculate_total_localization_time', return_value=0)    
    @patch('extractor.filter_threshold', return_value=True)
    def test_apply_threshold_on_statistics(self, mock_filter, mock_lcsn_time, mock_movement_time, mock_duration):
        extractor.generate_statistics(os.path.join(os.getcwd(),"fixtures/formatted_file.csv"))
        assert mock_lcsn_time.called
        assert mock_movement_time.called
        assert mock_duration.called    

    @patch('extractor.calc_time_saved_percentage', return_value=0)
    def test_compare_two_stats(self, mock_saved_percentage):

        result_1 = ("0:00:01.000000","0:00:01.000000","0:00:01.000000","0:00:01.000000")

        result_2 = ("0:00:01.000000","0:00:01.000000","0:00:01.000000","0:00:01.000000")
        t1, t2, t3 = extractor.compare_two_stats(result_1, result_2)
        self.assertEqual(0, t1)
        self.assertEqual(0, t2)
        self.assertEqual(0, t3)

if __name__ == "__main__":
    unittest.main()
