import os

# ---- MAP PARAMETERS ----
PLACE = "Manhattan, New York, USA"
MIN_STOP_SPACING_M = 150        # keep stops at least this far apart
HIGHWAY_BUFFER_M = 10           # how far from those roads to exclude
NETWORK_TYPE = "drive"          # use "drive" for pickup stops on roads

PARENT_DIR = os.path.dirname(os.getcwd())
DATA_DIR = "Data"
TAXI_ZONE_DIR = "Data/taxi_zones"
RILEY_DIR = "Data/manhattan-network"
STOP_DIR = "Data/stops"
DAY_DIR = "Data/days"
TRANSFORM_DAY_DIR = "Data/transform_days/"
OUTPUT_DIR = PARENT_DIR + "/Outputs/"

DATES_2015 = ['07-06', '07-15', '08-04', '08-28', '09-17', '09-26', '10-08',
              '10-25', '11-10', '11-30', '12-11', '12-30']
DATES_2016 = ['01-09', '01-29', '02-22', '02-25', '03-16', '03-29', '04-01',
              '04-27', '05-12', '05-21', '06-13', '06-28']

name_to_instance = {
    '20151025': 'V1200_R582',
    '20150926': 'V1200_R646',
    '20160109': 'V1300_R655',
    '20151230': 'V800_R660'
}

TEST_SIZE = {"small": [0,40000],
             "medium":[40000,50000],
             "large":[50000,70000]}

TEST_GROUP = {"small": "< 40,000",
              "medium": "40,000 - 50,000",
              "large": "50,000 < "}

FOLDERS = {
    'Phase_2': {
        'nbPicks': "1-nbPicks",
        'pruning': "2-pruning",
        'truncate': "3-truncate",
        'dropPick': "4-dropPick",
        'dynamic': "5-dynamic",
        'ablation': "6-ablation",
        'commit': "7-commit",
        'multiObj': "8-multiObj",
        'customer_weight': "9-customer_weight",
        'exact': "10-Exact_Full",
        'compare': "11-compare",
    },
    'Phase_3': {
        'return': "1.return",
        'partial': "2.partial",
        'initialDual': "3.dual",
        'batch': "4.batch",
        'compare': "5.compare",
    }
}

param_to_setting = {
        'Ab_truncate_1': 'Without Truncation',
        'Ab_truncate_0': 'Without Truncation',
        'Ab_drop_1': 'Drop-Pick Allowed',
        'Ab_drop_0': 'Drop-Pick Allowed',
        'Ab_dynamic_0': '2 Pickups (fix)',
        'Ab_dynamic_1': '2 Pickups (fix)',
        'no_commit_0': 'Full Configuration',
        'no_commit_1': 'Full Configuration'
    }

instance_category = {
    "V1200_R35576": "4.High Demand",
    "V1300_R35576": "3.Tight Supply",
    "V1400_R35576": "2.Balanced Supply",
    "V1500_R35576": "1.Excess Supply",

    "V1200_R35725": "4.High Demand",
    "V1300_R35725": "3.Tight Supply",
    "V1400_R35725": "2.Balanced Supply",
    "V1500_R35725": "1.Excess Supply",

    "V800_R25674": "4.High Demand",
    "V900_R25674": "3.Tight Supply",
    "V1000_R25674": "2.Balanced Supply",
    "V1100_R25674": "1.Excess Supply",

    "V1300_R37722": "4.High Demand",
    "V1400_R37722": "3.Tight Supply",
    "V1500_R37722": "2.Balanced Supply",
    "V1600_R37722": "1.Excess Supply",
}

# Define columns for each data type
DATA_TYPE_COLUMNS = {
    'request': [
        'WaitTime',
        'TripDelay',
        'CommitWaitTime',
        'AssignTime',
    ],
    'vehicle': [
        'idleTime',
        'serviceTime',
        'driveFullTime',
        'driveEmptyTime',
        'returnEmptyTime',
        '#Stops',
        '#RequestsServed',
        'WaitTime',
        'TripDelay',
    ],
    'time': [
        'Epoch',
        'nbRequests',
        'nbNodes',
        'EpochRuntime',
        'ElapsedTime'
        'totalColumn',
        '#LGenerated',
        '#ColumnsAdded',
        'nbNegative',
        'meanDual',
        '#Idle',
        '#passPerVehicle',
        '#requestPerVehicle',
        '#nodePerVehicle',
        'totalRoutes',
    ],
}

DATA_TYPE_FILES = {
    'request': "Requests_{mode}.csv",
    'vehicle': "Vehicles_{mode}.csv",
    'time':    "epochRuntime_{mode}.csv",
}

vehicle_groups = [
    "1.Excess Supply",
    "2.Balanced Supply",
    "3.Tight Supply",
    "4.High Demand",
]

vehicle_groups_labels = [
    "Excess Supply",
    "Balanced Supply",
    "Tight Supply",
    "High Demand",
]

OBJECTIVES = [
    "wait = 1, delay = 0",
    "wait = 1, delay = 0.5",
    "wait = 1, delay = 1",
]

OBJECTIVES_labels = [
    "$W_{wait}$ = 1, $W_{delay}$ = 0",
    "$W_{wait}$ = 1, $W_{delay}$ = 0.5",
    "$W_{wait}$ = 1, $W_{delay}$ = 1",
]

MODE_DICT = {
        'STATIC': 'S',
        'ANYTIME': 'A',
        'DYNAMIC': 'D',
    }