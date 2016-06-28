# race environment for nlab

require pynlab, Box2D, pygame

````
usage: python main.py [-h] [-p name] [-c n] [file]

positional arguments:
  file                  map file (default: default.json)

optional arguments:
  -h, --help            show this help message and exit
  -p name, --pipe name  pipe name (default: nlab)
  -c n, --count n       number of players (default: 25)

````

Also you can launch in "manual" mode. To do this run `race_env.py` file.
Controls - Arrow keys. Press Enter to restart environment.

````
usage: python race_env.py [-h] [file]

positional arguments:
  file        map file (default: default.json)

optional arguments:
  -h, --help  show this help message and exit
````

###Screenshot of custom map:
![screenshot](./screenshot.png)