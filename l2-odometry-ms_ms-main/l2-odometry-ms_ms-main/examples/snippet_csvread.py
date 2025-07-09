import csv

with open('./csv/odom_forward.csv') as csvfile:
    reader = csv.DictReader(csvfile, delimiter=';', quotechar='|')
    for row in reader:        
        print(row['pos_l'], row['pos_r[ticks]'])
