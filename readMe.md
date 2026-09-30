idf.py create-project earthquake_alert
idf.py set-target esp32c3

-----

idf.py build
idf.py -p COM11 flash
idf.py -p COM11 monitor

idf.py -p COM11 flash monitor