import os

DATES_2015 = ['07-06', '07-15', '08-04', '08-28', '09-17', '09-26', '10-08',
              '10-25', '11-10', '11-30', '12-11', '12-30']
DATES_2016 = ['01-09', '01-29', '02-22', '02-25', '03-16', '03-29', '04-01',
              '04-27', '05-12', '05-21', '06-13', '06-28']

NB_VEHICLES = [500, 600, 700, 800, 900, 1000, 1100, 1200, 1300, 1400, 1500, 1600, 1700, 1800, 1900, 2000, 2500, 3000,
               4000]
CAPACITY = [3, 4, 5, 6, 7, 8, 9, 10]
OS = "osx"
DAYS_DIR = "NYC/days/"
VEHICLES_DIR = "NYC/manhattan-vehicles/"
NETWORK_DIR = "NYC/manhattan-network/"
PARENT_DIR = os.path.dirname(os.getcwd())
DATASETS_DIR = PARENT_DIR + "/DARP_IPS/datasets/"
OUT_BOUND = 999

WIN_TITLE_SIZE = 25
WIN_TEXT_SIZE = 20
WIN_FIG_SIZE = (30, 18)
OS_TITLE_SIZE = 15
OS_TEXT_SIZE = 8
OS_FIG_SIZE = (15, 9)
OS_MAP_SIZE = (13, 7)

LIMITED_DISTRICTS = [189, 131, 280, 199]
RESTRICTED_DISTRICTS = [189, 131, 280, 199, 52, 54, 57, 100, 119, 127, 168, 174, 206, 207, 266, 277, 286,
                        287, 290, 291, 299, 87, 178, 9, 98, 84, 129, 192]

INBOUND_DISTRICTS = [189, 131, 280, 199, 52, 54, 57, 100, 119, 127, 168, 174, 206, 207, 266, 277, 286,
                     287, 290, 291, 299, 87, 178, 9, 98, 84, 129, 142, 192, 296]

OUT_BOUND_DISTRICTS = [91, 50, 180, 172, 117, 257, 247]