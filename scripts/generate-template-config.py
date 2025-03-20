import json

def load_config(file_path):
    with open(file_path, 'r') as file:
        return json.load(file)

def generate_view_data(config):
    views = []
    max_views = config["config"]["max_views"]
    gauges_per_view = config["config"]["gauges_per_view"]
    
    for _ in range(max_views):
        for setting in config["view"]:
            print(setting["cmd"])
        view = {
            "enabled": True,
            "num_gauges": gauges_per_view,
            "background": "Flare",
            "gauges": [
                {"pid": "Boost", "theme": "Grump Cat"},
                {"pid": "Oil Temp", "theme": "Stock ST"},
                {"pid": "RPM", "theme": "Grump Cat"}
            ]
        }
        views.append(view)
    
    return views

def generate_alert_data(config):
    alerts = []
    max_alerts = config["config"]["max_alerts"]
    
    for _ in range(max_alerts):
        alert = {
            "enabled": True,
            "background": "Flare",
            "gauges": [
                {"pid": "Boost", "theme": "Grump Cat"},
                {"pid": "Oil Temp", "theme": "Stock ST"},
                {"pid": "RPM", "theme": "Grump Cat"}
            ]
        }
        alerts.append(alert)
    
    return alerts

def generate_dynamic_data(config):
    return [
        {"priority": "Highest", "view_index": 2, "enabled": True, "pid": "Oil", "thresh": 200, "compare": ">"},
        {"priority": "Medium", "view_index": 1, "enabled": True, "pid": "Boost", "thresh": 10, "compare": ">"},
        {"priority": "Default", "view_index": 0, "enabled": True, "pid": "N/A", "thresh": 999999, "compare": "N/A"}
    ]

def generate_output_json(config):
    output = {
        "view": generate_view_data(config),
        "alert": generate_alert_data(config),
        "dynamic": generate_dynamic_data(config)
    }
    return output

def main():
    config = load_config("config.json")
    output_json = generate_output_json(config)
    
    with open("output.json", "w") as file:
        json.dump(output_json, file, indent=4)
    
    print("Generated output.json successfully!")

if __name__ == "__main__":
    main()