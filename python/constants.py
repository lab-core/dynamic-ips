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

TEST_GROUP = {"small": "1.< 40K",
              "medium": "2.40K-50K",
              "large": "3.50K < "}

FOLDERS = {
    'Phase_1': {
        'compare': "1-compare",
    },
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
        'obj_compare': "12-obj_compare",
        'other': "Instances_2h-7",
    },
    'Phase_3': {
        'rebalance': "1-rebalance",
        'profile': "2-profile",
        'reOptimize': "3-reOptimize",
        'prcocess': "4-prcocess",
        'compare': "5-compare",
        'shuttle': "6-shuttle",
        'rebalance_anytime': "7-rebalance_anytime",
        'anytime': "8-anytime",
    },
    'Phase_4': {
        'isud': "1-isud",
    }

}

param_to_setting = {
        'Ab_truncate_1': 'Without Truncation',
        'Ab_truncate_0': 'Without Truncation',
        'Ab_drop_1': 'Drop-Pick Allowed',
        'Ab_drop_0': 'Drop-Pick Allowed',
        'Ab_dynamic_0': '2 Pickups (fix)',
        'Ab_dynamic_1': '2 Pickups (fix)',
        'commit_0': 'Full Configuration',
        'commit_1': 'Full Configuration'
    }

param_to_setting_labels = [
    'Without Truncation',
    'Drop-Pick Allowed',
    '2 Pickups (fix)',
    'Full Configuration'
    ]

Supply_scenarios = {
    # NYC_DARP_Benchmark

    # G1: R68889 with vehicle_counts (1400 1500 1600 1700)
    "V1400_R68889": "4.High Demand",
    "V1500_R68889": "3.Tight Supply",
    "V1600_R68889": "2.Balanced Supply",
    "V1700_R68889": "1.Excess Supply",

    # G2: 72353 with vehicle_counts (1450 1550 1650 1750)
    "V1450_R72353": "4.High Demand",
    "V1550_R72353": "3.Tight Supply",
    "V1650_R72353": "2.Balanced Supply",
    "V1750_R72353": "1.Excess Supply",

    # G3: R68462, R64942 with vehicle_counts (1300 1400 1500 1600)
    "V1300_R68462": "4.High Demand",
    "V1400_R68462": "3.Tight Supply",
    "V1500_R68462": "2.Balanced Supply",
    "V1600_R68462": "1.Excess Supply",

    "V1300_R64942": "4.High Demand",
    "V1400_R64942": "3.Tight Supply",
    "V1500_R64942": "2.Balanced Supply",
    "V1600_R64942": "1.Excess Supply",

    # Riley_Benchmark
    "V1200_R35576": "4.High Demand",
    "V1300_R35576": "3.Tight Supply",
    "V1400_R35576": "2.Balanced Supply",
    "V1500_R35576": "1.Excess Supply",

    "V1300_R37722": "4.High Demand",
    "V1400_R37722": "3.Tight Supply",
    "V1500_R37722": "2.Balanced Supply",
    "V1600_R37722": "1.Excess Supply",

    "V1200_R35725": "4.High Demand",
    "V1300_R35725": "3.Tight Supply",
    "V1400_R35725": "2.Balanced Supply",
    "V1500_R35725": "1.Excess Supply",

    "V800_R25674": "4.High Demand",
    "V900_R25674": "3.Tight Supply",
    "V1000_R25674": "2.Balanced Supply",
    "V1100_R25674": "1.Excess Supply",
}

# Define columns for each data type
DATA_TYPE_COLUMNS = {
    'request': [
        'WaitTime',
        'TripDelay',
        'CommitWaitTime',
        'AssignTime',
        'ReadyTime'
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
        'nbNewRequests',
        'nbNodes',
        'EpochRuntime',
        'ElapsedTime'
        'totalColumn',
        '#LGenerated',
        '#ColumnsAdded',
        'nbNegative',
        'meanDual',
        'maxDual'
        '#Idle',
        '#passPerVehicle',
        '#requestPerVehicle',
        '#nodePerVehicle',
        'totalRoutes',
        'waitTime',
        'TripDelay',
        'LinearObjective',
        'Objective',
        '#SP Iter',
    ],
    'epochVehicle': [
        'Epoch',
        'nbOnboards',
        'nbCommitted',
        'nbRequests',
        'nbNodes',
        'length'
        'avgPassPerStop',
        'totalTripDelay',
        'totalWait',
    ],
}

DATA_TYPE_FILES = {
    'request': "Requests_{mode}.csv",
    'vehicle': "Vehicles_{mode}.csv",
    'time':    "epochRuntime_{mode}.csv",
    'epochVehicle': "VehicleEpoch_{mode}.csv",
}

vehicle_groups = [
    "1.Excess Supply",
    "2.Balanced Supply",
    "3.Tight Supply",
    "4.High Demand",
]

customer_groups = [
    "1.< 40K",
    "2.40K-50K",
    "3.50K < "]

customer_groups_labels = [
    "< 40K",
    "40K-50K",
    "50K < "]

vehicle_groups_labels = [
    "Excess Supply",
    "Balanced Supply",
    "Tight Supply",
    "High Demand",
]

vehicle_groups_labels2 = [
    "ES",
    "BS",
    "TS",
    "HD",
]

OBJECTIVES = [
    "wait = 1, delay = 0",
    "wait = 1, delay = 0.5",
    "wait = 1, delay = 1",
]

OBJECTIVES2 = [
    "delay = 0.5, excess = 0, Relative = 0",
    "delay = 1, excess = 0, Relative = 0",
    "delay = 1, excess = 1, Relative = 0",
    "delay = 0.5, excess = 0, Relative = 1",
]

OBJECTIVES_labels = [
    r"$\omega_{wait}$ = 1, $\omega_{drop}$ = 0",
    r"$\omega_{wait}$ = 1, $\omega_{drop}$ = 0.5",
    r"$\omega_{wait}$ = 1, $\omega_{drop}$ = 1",
]

OBJECTIVES_labels3 = [
    r"$\omega_{drop}$ = 0.5, $\lambda_{excess}$ = 1, $\lambda^i_{normal}$ = 1",
    r"$\omega_{drop}$ = 1, $\lambda_{excess}$ = 1, $\lambda^i_{normal}$ = 1",
    r"$\omega_{drop}$ = 1, $\lambda_{excess}$ = 0, $\lambda^i_{normal}$ = 1",
    r"$\omega_{drop}$ = 0.5, $\lambda_{excess}$ = 1, $\lambda^i_{normal}$ = $t_i$",
]

OBJECTIVES_labels2 = [
    r"B-CG-C $(\omega_{drop}$ = 0.5)",
    r"B-CG-C $(\omega_{drop}$ = 1)",
    "Total service time",
    "Normalized B-CG-C",
]

MODE_DICT = {
        'STATIC': 'S',
        'ANYTIME': 'A',
        'DYNAMIC': 'D',
    }

Blue_Palette = ["#B4C6CC", "#26619C", "#0F2E3A", "#2B4363", "#6DAED5" , "#4783B1"]