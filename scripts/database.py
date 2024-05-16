#!/usr/bin/env python3
import mysql.connector
import pandas as pd
from math import radians, sin, cos, sqrt, atan2

class db:
    def __init__(self):
        self.conn = mysql.connector.connect(
            host="flymate.clzinzr5ga32.eu-west-2.rds.amazonaws.com",
            user="admin",
            password="admin12345",
            database="robot_db"
        )

        self.cursor = self.conn.cursor()

    def insert_signup_data(self, data):
        self.data = data
        print(data)
        for row in self.data:
            insert_query = """
            INSERT INTO robot_db.signup_table
            (firstname, lastname, postcode, username, password)
            VALUES (%s, %s, %s, %s, %s)
            """
            values = (
                row['firstname'],
                row['lastname'],
                row['postcode'],
                row['username'],
                row['password']
            )
            self.cursor.execute(insert_query, values)
        self.conn.commit()



    def check_user_exists(self, username, password):
        query = "SELECT * FROM signup_table WHERE username = %s AND password = %s"
        self.cursor.execute(query, (username, password))
        rows = self.cursor.fetchall()

        if rows:
            data = []
            for row in rows:
                username_data = {
                    "firstname": row[0],
                    "lastname": row[1],
                    "postcode": row[2],
                    "username": row[3],
                    "password": row[4]
                }
                data.append(username_data)
            self.user_data = pd.DataFrame(data)
            return True
        else:
            print("User not found")
            return False
        
    def return_data(self):
        return self.user_data["username"]
  

    def check_add_friend(self, username):
        query = "SELECT * FROM signup_table WHERE username = %s"
        self.cursor.execute(query, (username,))
        rows = self.cursor.fetchall()
        if rows:
            data=[]
            for row in rows:
                friend_data = {
                    "firstname": row[1],
                    "lastname": row[2],
                    "postcode": row[3],
                    "username": row[4],
                    "password": row[5]
                }
                data.append(friend_data)
            self.friend_data = pd.DataFrame(data)
            return True 
        else:   
            print("User not found")
            return False


    def return_friend_data(self):
        return self.friend_data

    def add_friend(self, username, friend):
        query = "SELECT * FROM robot_db.friend_table WHERE username = %s AND friend = %s"
        values = (username, friend)
        self.cursor.execute(query, values)
        row = self.cursor.fetchone()
        
        if row:
            print("You and " + friend + " are already friends")
            return True 
        else:   
            query = "INSERT INTO robot_db.friend_table (username, friend) VALUES (%s, %s)"
            values = (username, friend)
            self.cursor.execute(query, values)
            self.conn.commit()
            return False

    def fetch_friends(self, username):
        query = "SELECT * FROM robot_db.friend_table WHERE username = %s"
        self.cursor.execute(query, (username,))
        rows = self.cursor.fetchall()
        data=[]
        for row in rows:
            friend_list = {
                "friend": row[2],
            }
            data.append(friend_list)
        self.friend_data = pd.DataFrame(data)
        return self.friend_data


    def save_order_details(self, username, destination_username, package_weight, package_dimentions, tracking_choice):
        query = "INSERT INTO robot_db.order_table (username, destination_username, package_weight, package_dimentions, tracking_choice) VALUES (%s, %s, %s, %s, %s)"
        values = (username, destination_username, package_weight, package_dimentions, tracking_choice)
        self.cursor.execute(query, values)

        query = "SELECT postcode FROM robot_db.signup_table WHERE username = %s"
        self.cursor.execute(query, (username,))
        sending_postcode = self.cursor.fetchone()

        query = "SELECT postcode FROM robot_db.signup_table WHERE username = %s"
        self.cursor.execute(query, (destination_username,))
        destination_postcode = self.cursor.fetchone()

        return sending_postcode, destination_postcode
    
    def calculate_distance(self, lat1, lon1, lat2, lon2):
        R = 6371.0 

        lat1 = float(lat1)
        lon1 = float(lon1)
        lat2 = float(lat2)
        lon2 = float(lon2)
        lat1_rad = radians(lat1)
        lon1_rad = radians(lon1)
        lat2_rad = radians(lat2)
        lon2_rad = radians(lon2)


        dlon = lon2_rad - lon1_rad
        dlat = lat2_rad - lat1_rad


        a = sin(dlat / 2)**2 + cos(lat1_rad) * cos(lat2_rad) * sin(dlon / 2)**2
        c = 2 * atan2(sqrt(a), sqrt(1 - a))
        distance = R * c

        return distance


    def find_nearest_charging_station(self, sending_coords):
        sending_lat, sending_lon = sending_coords


        query = """
            SELECT name, latitude, longitude 
            FROM robot_db.station_table
        """
        self.cursor.execute(query)
        nearest_station = None
        min_distance = float('inf')


        for row in self.cursor:
            station_name, station_lat, station_lon = row
            distance = self.calculate_distance(sending_lat, sending_lon, station_lat, station_lon)


            if distance < min_distance:
                nearest_station = station_name
                charging_coords = (station_lat, station_lon)
                min_distance = distance

        return charging_coords
            




