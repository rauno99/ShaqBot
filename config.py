from configparser import ConfigParser

config = ConfigParser()
config.read("config.ini")

if not "colors" in config.sections():
    config["colors"] = { }


def get_color_range(color):
    values = config["colors"][color].split(",")
    values = [int(value) for value in values]

    return values[:3], values[3:]


def set_color_range(color, minimum, maximum):
    values = [str(i) for i in sum((minimum, maximum), [])]
    config["colors"][color] = ",".join(values)
    save()


def save():
    with open("config.ini", "w") as file:
        config.write(file)
