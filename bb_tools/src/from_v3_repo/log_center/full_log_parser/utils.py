NUM_VAR_FOR_EACH_MOVEMENT = 9 #  To help with formatting in CSV file
MAX_COL = 59
MARK_COL = MAX_COL - 3
MAX_NUM_STEPS = 5 


#  SPECIFIC COLUMNS
TR_X = 1
TR_Y = 2
TR_RZ = 3
AC_X = 4
AC_Y = 5
AC_RZ = 6
ACC_COL = 9
LCSN_COL = 8
MOVETIME_COL = 7 # or 8
STARTEND_COL = MAX_COL - 2
TIMECHECK_COL = MAX_COL - 1 # 58
MOVEMENT_NUM_COL = MAX_COL - 4

# REGEX
keyword_start = r'(nav: Goal accepted, id: )(\d+)(, coords:) (\(-?\d+\.\d+, -?\d+\.\d+, -?\d+\.\d+\))'

#(?P<time_date>\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}.\d{3})

keyword_tr_pos = r'(gbm: Got new goal, internal id: )(\d+)(, at: )(\(-?\d+\.\d+, -?\d+\.\d+\), rz: -?\d+\.\d+)'

keyword_move_base_start = r'(nav: move_base actived.)'

keyword_move_base_end = r'(nav: Move base done to BoothBotNavigationState.MOVING)'

keyword_start_localisation = r'(nav: GBM actived.)'

keyword_end_localisation = r'(gbm: Base estimated pose are )(\(\(-?\d+.\d+, -?\d+.\d+\), -?\d+.\d+\))'

keyword_finish = r'(BBC: goal \d+ finished)'

keyword_last_localisation = r'(nav: Localization takes \d+.\d+s)'

keyword_marked = r'(BBC: Marking\.\.\.)'

desired_files_reg = [
        r'(.*boothbot\_controller\-.*\.log)',
        r'(.*gbm-2.log)'
    ]

regex_list = [
    ".*\[(ros.*)\]\[((INFO|DEBUG|ERROR|WARNING))\] (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):( nav: Goal accepted, id: -?\d+\, coords: \(-?\d+\.?\d+, -?\d+\.?\d+, -?\d+\.?\d+\))",
    
    ".*\[(ros.*)\]\[((INFO|DEBUG|ERROR|WARNING))\] (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):( nav: GBM actived.)",
    
    ".*\[(ros.*)\]\[((INFO|DEBUG|ERROR|WARNING))\] (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):( gbm: Got new goal, internal id: \d+, at: \(-?\d+.\d+, -?\d+.\d+\), rz: -?\d+.\d+)",
    
    ".*\[(ros.*)\]\[((INFO|DEBUG|ERROR|WARNING))\] (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):( nav: move_base actived.)",
    
    ".*\[(ros.*)\]\[((INFO|DEBUG|ERROR|WARNING))\] (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):( nav: Move base done to BoothBotNavigationState.MOVING)",
    
    ".*\[(ros.*)\]\[((INFO|DEBUG|ERROR|WARNING))\] (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):( gbm: Base estimated pose are \(\(-?\d+.\d+, -?\d+.\d+\), -?\d+.\d+\))",

    ".*\[(ros.*)\]\[((INFO|DEBUG|ERROR|WARNING))\] (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):( nav: Localization takes -?\d+.\d+s)",
    
    ".*\[(ros.*)\]\[((INFO|DEBUG|ERROR|WARNING))\] (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):( BBC: Marking\.\.\.)",
    
    ".*\[(ros.*)\]\[((INFO|DEBUG|ERROR|WARNING))\] (\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2},\d{3}):( BBC: goal \d+ finished)"
]

# Dict of expected REGEX

expected_regex = {

    'keyword_start': [
            str(keyword_start_localisation),
            str(keyword_tr_pos)
            ],
    'keyword_start_localisation' : [
            str(keyword_tr_pos),
            str(keyword_end_localisation),
            str(keyword_move_base_start) # ignore if haven new goaled
            ],
    'keyword_tr_pos' : [
            str(keyword_move_base_start),
            str(keyword_start_localisation),
            ],
    'keyword_move_base_start' : [
            str(keyword_move_base_end)
            ],
    'keyword_move_base_end' : [
            str(keyword_start_localisation),
            str(keyword_last_localisation),
            str(keyword_move_base_start) #  For consecutive movement
            ],
    'keyword_last_localisation' : [
            str(keyword_marked)
            ],
    #  Base estimated
    'keyword_end_localisation' : [
            str(keyword_last_localisation),
            str(keyword_start_localisation),
            str(keyword_end_localisation),
            str(keyword_tr_pos)
            ],
    'keyword_marked' : [
            str(keyword_finish),
            str(keyword_marked)        
            ],
    'keyword_finish' : [
            str(keyword_start)
            ],
    'keyword_start_localisation_no_goal' : [
            str(keyword_tr_pos),
            str(keyword_end_localisation),
            ],
}

table_col = ['Point',
        'tr_x','tr_y','tr_rz','ac_x','ac_y','ac_rz','MoveTime','Localisation Time', 'Accuracytal_(dist apart),m',
        'tr_x','tr_y','tr_rz','ac_x','ac_y','ac_rz', 'MoveTime','Localisation Time','Accuracy(dist apart),m',
        'tr_x','tr_y','tr_rz','ac_x','ac_y','ac_rz', 'MoveTime','Localisation Time','Accuracy(dist apart),m',
        'tr_x','tr_y','tr_rz','ac_x','ac_y','ac_rz', 'MoveTime','Localisation Time','Accuracy(dist apart),m',
        'tr_x','tr_y','tr_rz','ac_x','ac_y','ac_rz', 'MoveTime','Localisation Time','Accuracy(dist apart),m',
        'tr_x','tr_y','tr_rz','ac_x','ac_y','ac_rz', 'MoveTime','Localisation Time','Accuracy(dist apart),m',
        'Moves', 'TotalMarkingTime', 'Startend_time', 'TimeCheck'
        ]

extracted_keywords_file = "unsorted_extracted_log.txt"
sorted_extracted_file = "sorted_extracted_log.csv"
preprocessed_file = "preprocessed.csv"
formatted_file = "formatted.csv"
exception_file = "exceptions.csv"
log_path = "/tmp/extraction.log"