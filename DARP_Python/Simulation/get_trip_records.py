import os, json, time, requests
from datetime import datetime
from urllib3.util.retry import Retry
from requests.adapters import HTTPAdapter
import constants as c
import random


BASE_URL_2016 = "https://data.cityofnewyork.us/resource/uacg-pexx.json"
BASE_URL_2015 = "https://data.cityofnewyork.us/resource/2yzn-sicd.json"

APP_TOKEN = os.getenv("SODA_APP_TOKEN")  # set this in your env!
LIMIT = 50000
PAGE_PAUSE_SEC = 2   # gentle pacing between pages


def make_session():
    s = requests.Session()
    # Retry on 429, 500, 502, 503, 504, with backoff.
    retries = Retry(
        total=8,
        connect=3,
        read=3,
        backoff_factor=1.2,             # exponential backoff
        status_forcelist=(429, 500, 502, 503, 504),
        allowed_methods=frozenset(["GET"])
    )
    s.mount("https://", HTTPAdapter(max_retries=retries))
    if APP_TOKEN:
        s.headers["X-App-Token"] = APP_TOKEN
    # Optional: keep-alive hint
    s.headers["Connection"] = "keep-alive"
    return s

def fetch_data_for_date(date_str):
    year = int(date_str[:4])
    if year == 2016:
        base_url = BASE_URL_2016
        field = "tpep_pickup_datetime"
        start = f"{date_str}T00:00:00.000"
        end   = f"{date_str}T23:59:59.999"
    elif year == 2015:
        base_url = BASE_URL_2015
        field = "pickup_datetime"
        start = f"{date_str}T00:00:00"
        end   = f"{date_str}T23:59:59"
    else:
        raise ValueError("Only 2015 and 2016 supported.")

    session = make_session()
    offset = 0
    all_data = []  # Collect all data in memory like the second script
    out_path = os.path.join(c.DAY_DIR, f"{date_str}_manhattan.json")  # Use DATA_DIR and .json extension

    while True:
        params = {
            "$select": "*",  # narrow if you can for speed
            "$where": f"{field} between '{start}' and '{end}'",
            "$order": ":id",         # stable pagination
            "$limit": LIMIT,
            "$offset": offset,
        }
        if APP_TOKEN:
            params["$$app_token"] = APP_TOKEN

        try:
            resp = session.get(base_url, params=params, timeout=90)
            # Respect server-provided cooldowns
            if resp.status_code in (429, 500, 502, 503, 504):
                retry_after = resp.headers.get("Retry-After")
                if retry_after:
                    try:
                        time.sleep(float(retry_after))
                    except ValueError:
                        time.sleep(5)
                else:
                    time.sleep(5)
                # Let Retry handle re-try on next loop iteration
                resp.raise_for_status()

            resp.raise_for_status()
            batch = resp.json()
        except requests.RequestException as e:
            # Last-resort pause; then continue and let Retry logic keep working
            print(f"Warn: {e}. Pausing 10s before continuing…")
            time.sleep(10)
            continue

        if not batch:
            break

        all_data.extend(batch)  # Add batch to the main list
        offset += LIMIT

        print(f"[{date_str}] {len(all_data):,} rows (last page size {len(batch):,})")
        time.sleep(PAGE_PAUSE_SEC+ random.uniform(0, 1))  # be nice between pages

    # Save to JSON with indentation like the second script
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(all_data, f, indent=4, ensure_ascii=False)

    print(f"✅ Saved {len(all_data):,} rows for {date_str} to {out_path}")

"""
Example:
----------
    for date in ut.create_dates():
        fetch_data_for_date(date_str= date)
"""