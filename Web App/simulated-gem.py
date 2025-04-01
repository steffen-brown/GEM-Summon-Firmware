import requests
import time

API_BASE = "https://gem-summon-backend.onrender.com"
TOKEN = "sfdvghtrgta4y5tes4srt4awesrfg"
HEADERS = {"Content-Type": "application/json", "Authorization": TOKEN}

def get_phone_location():
    """Fetch the current summon location from the API."""
    try:
        response = requests.get(f"{API_BASE}/latest-location", headers=HEADERS)
        if response.status_code == 200:
            data = response.json()
            return data.get("location")
        return None
    except Exception as e:
        print("Error fetching phone location:", e)
        return None

def update_car_location(location):
    """Update the car's location via the API."""
    try:
        response = requests.post(f"{API_BASE}/car-location", headers=HEADERS, json={"location": location})
        if response.status_code != 200:
            print("Error updating car location:", response.text)
    except Exception as e:
        print("Exception updating car location:", e)

def update_car_status(status):
    """Update the car's status via the API."""
    try:
        response = requests.post(f"{API_BASE}/car-status", headers=HEADERS, json={"status": status})
        if response.status_code != 200:
            print("Error updating car status:", response.text)
    except Exception as e:
        print("Exception updating car status:", e)

def simulate_movement(start, end, duration=10, steps=20):
    """
    Simulate moving from start to end over 'duration' seconds in 'steps' increments.
    Returns the new (end) location.
    """
    lat_diff = end["lat"] - start["lat"]
    lng_diff = end["lng"] - start["lng"]
    step_time = duration / steps
    new_location = {"lat": start["lat"], "lng": start["lng"]}
    for i in range(1, steps + 1):
        new_location["lat"] = start["lat"] + (lat_diff * i / steps)
        new_location["lng"] = start["lng"] + (lng_diff * i / steps)
        update_car_location(new_location)
        print(f"Moving to: {new_location}")
        time.sleep(step_time)
    return new_location

def main():
    # Initial car position and status
    car_location = {"lat": 40.092857, "lng": -88.235992}
    update_car_location(car_location)
    update_car_status("IDLE")
    last_phone_location = get_phone_location()

    print("Car simulation started. Current status: IDLE")
    
    while True:
        phone_location = get_phone_location()
        if phone_location and phone_location != last_phone_location:
            print("New summon location detected:", phone_location)
            last_phone_location = phone_location

            # Change status to SUMMONING and simulate movement over 10 seconds
            update_car_status("SUMMONING")
            car_location = simulate_movement(car_location, phone_location, duration=25, steps=20)

            # Once arrived, update status to ARRIVED
            update_car_status("ARRIVED")
            print("Arrived at summon location:", car_location)
        time.sleep(2)

if __name__ == "__main__":
    main()
