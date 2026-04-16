from jinja2 import Environment, FileSystemLoader

# Setup Jinja2 environment
env = Environment(loader=FileSystemLoader(searchpath="./templates"))
race_template = env.get_template("docker-compose_race.j2")
num_clients = 1
race_compose_name = f"docker-compose_race_{num_clients}_cars.yml"
initial_gaps_m = 30


race = {"number_of_racecars": num_clients,
        "orchestrator":
            {
                "ip":"10.6.1.1"
            },
        "ctun":
            {  
                "ip":"10.6.1.2"
            },
        "lichtblick":
            {
                "ip":"10.6.1.3"
            }
        }


# Generate properties of the individual race cars. These parameters change between the different instances of a race car
race_participants = list()
for i in range(race["number_of_racecars"]):
     race_car_id = (i + 1) * 10
     race_participants.append(
        {
            "race_car": {"name":f"car{ i + 1 }_veos","ID": race_car_id , "ip": f"10.6.{ race_car_id }.1","s": f"{i * initial_gaps_m}"},
            "vesi":  {"name":f"car{ i + 1 }_vesi", "ip": f"10.6.{ race_car_id }.2"},
            "bridge":  {"name":f"car{ i + 1 }_dspace_bridge", "ip": f"10.6.{ race_car_id }.3"},
            "foxglove":  {"name":f"car{ i + 1 }_foxglove_bridge", "ip": f"10.6.{ race_car_id }.4", "port":f"87{ race_car_id }"},
            "ROS":  {"DOMAIN_ID": race_car_id },
            "stack":  {"name":f"car{ i + 1 }_demo_stack", "ip":f"10.6.{ race_car_id }.5"},
            "can_netns": {"name": f"car{ i + 1 }_can_netns", "ip": f"10.6.{ race_car_id }.6"}
        }
     )

# Render the template
race_docker_compose = race_template.render(race=race, race_participants=race_participants)

# Save to docker-compose.yml
with open(race_compose_name, "w") as f:
    f.write(race_docker_compose)

print(f"{race_compose_name} generated successfully.")
