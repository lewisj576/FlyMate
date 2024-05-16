#!/usr/bin/env python3
import customtkinter as ctk
from database import db
from geopy.geocoders import Nominatim
import subprocess
import rospy



class App(ctk.CTk):
    def __init__(self, master=None, **kwargs):
        super().__init__(master, **kwargs)
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("dark-blue")
        self.signup_window()
        self.obj = db()

    def signup_window(self):
        self.geometry(f"{400}x{350}")
        self.title("Create Account Page")
        self.current_page = "signup"
        self.grid_columnconfigure((0, 1, 2, 3, 4, 5, 6), weight=1)
        self.first_name_label = ctk.CTkLabel(self, text="First Name:")
        self.first_name_label.grid(row=0, column=2, padx=10, pady=10, sticky="e")
        self.first_name_entry = ctk.CTkEntry(self)
        self.first_name_entry.grid(row=0, column=3, padx=10, pady=10, sticky="w")

        self.last_name_label = ctk.CTkLabel(self, text="Last Name:")
        self.last_name_label.grid(row=1, column=2, padx=10, pady=10, sticky="e")
        self.last_name_entry = ctk.CTkEntry(self)
        self.last_name_entry.grid(row=1, column=3, padx=10, pady=10, sticky="w")

        self.postcode_label = ctk.CTkLabel(self, text="Postcode:")
        self.postcode_label.grid(row=2, column=2, padx=10, pady=10, sticky="e")
        self.postcode_entry = ctk.CTkEntry(self)
        self.postcode_entry.grid(row=2, column=3, padx=10, pady=10, sticky="w")
        
        self.username_label = ctk.CTkLabel(self, text="Username:")
        self.username_label.grid(row=3, column=2, padx=10, pady=10, sticky="e")
        self.username_entry = ctk.CTkEntry(self)
        self.username_entry.grid(row=3, column=3, padx=10, pady=10, sticky="w")

        self.password_label = ctk.CTkLabel(self, text="Password:")
        self.password_label.grid(row=4, column=2, padx=10, pady=10, sticky="e")
        self.password_entry = ctk.CTkEntry(self)
        self.password_entry.grid(row=4, column=3, padx=10, pady=10, sticky="w")

        self.signup_button = ctk.CTkButton(self, text="Create Account", command=self.signup_button_press)
        self.signup_button.grid(row=5, column=2, columnspan=2, padx=10, pady=10, sticky="nsew")

        self.login_button = ctk.CTkButton(self, text="Login Page", command=self.login_button_press)
        self.login_button.grid(row=6, column=2, columnspan=2, padx=10, pady=10, sticky="nsew")

    def login_window(self):
        self.geometry(f"{400}x{200}")
        self.title("Login Page")
        self.current_page = "login"
        self.grid_columnconfigure((0, 1, 2, 3, 4, 5, 6), weight=1)
        self.username_label_login = ctk.CTkLabel(self, text="Username: ")
        self.username_label_login.grid(row=0, column=2, padx=10, pady=10, sticky="e")
        self.username_entry_login = ctk.CTkEntry(self)
        self.username_entry_login.grid(row=0, column=3, padx=10, pady=10, sticky="w")

        self.password_label_login = ctk.CTkLabel(self, text="Password:  ")
        self.password_label_login.grid(row=1, column=2, padx=10, pady=10, sticky="e")
        self.password_entry_login = ctk.CTkEntry(self)
        self.password_entry_login.grid(row=1, column=3, padx=10, pady=10, sticky="w")

        self.try_again_label_login = ctk.CTkLabel(self, text="")
        self.try_again_label_login.grid(row=7, column=2, columnspan=2)
    
    
    def signup_button_press(self):
        if self.current_page == "signup":
            
            first_name_val = self.first_name_entry.get()
            last_name_val = self.last_name_entry.get()
            postcode_val = self.postcode_entry.get()
            username_val = self.username_entry.get()
            password_val = self.password_entry.get()
            
            data = [{
                'firstname': first_name_val,
                'lastname': last_name_val,
                'postcode': postcode_val,
                'username': username_val,
                'password': password_val
            }]

            self.obj.insert_signup_data(data)
            self.login_button_press()
            
        else:
            self.username_label_login.grid_forget()
            self.password_entry_login.grid_forget()
            self.username_entry_login.grid_forget()
            self.password_label_login.grid_forget()
            self.signup_window()

    
    
    def login_button_press(self):
        if self.current_page == "login":
            self.username = self.username_entry_login.get()
            self.password = self.password_entry_login.get()
            print(self.username)
            print(self.password)
            check = self.obj.check_user_exists(self.username, self.password)
            if check == False:
                self.try_again_label_login.configure(text="Username or Password Incorrect Please Try Again")
            else:
                self.user_data = self.obj.return_data()
                self.signup_button.grid_forget()
                self.login_button.grid_forget()
                self.username_entry_login.grid_forget()
                self.password_entry_login.grid_forget()
                self.username_label_login.grid_forget()
                self.password_label_login.grid_forget()
                self.try_again_label_login.grid_forget()
                self.logged_in_window()

        else:
            self.first_name_label.grid_forget()
            self.last_name_label.grid_forget()
            self.first_name_entry.grid_forget()
            self.last_name_entry.grid_forget()
            self.postcode_label.grid_forget()
            self.postcode_entry.grid_forget()
            self.username_entry.grid_forget()
            self.username_label.grid_forget()
            self.password_entry.grid_forget()
            self.password_label.grid_forget()
            self.signup_button.configure(text="Create Account Page")
            self.login_button.configure(text="Login")
            self.login_window()

    def logged_in_window(self):
        self.geometry(f"{470}x{260}")
        self.title("Signed in as: " + self.username)
        self.grid_columnconfigure((0, 1, 2), weight=1)
        try:
            friend_list = self.obj.fetch_friends(self.username)
            options = []
            options.append("Select/Add")
            options = friend_list["friend"]
        except KeyError:
            print("user has no friends")
        
        self.title_logo = ctk.CTkLabel(self, text="DRONE A FRIEND", font=ctk.CTkFont(size=20, weight="bold"))
        self.title_logo.grid(row=0, column=1, pady=3, padx=3)
        self.dest_username_label = ctk.CTkLabel(self, text="Friends", anchor="center", font=ctk.CTkFont(size=10, weight="bold"))
        self.dest_username_label.grid(row=1, column=0, pady=3, padx=3)
        self.dest_username_option = ctk.CTkOptionMenu(self, values=options)
        self.dest_username_option.grid(row=2, column=0, pady=3, padx=3)
        self.add_friend_label = ctk.CTkLabel(self, text="New Friend", anchor="center", font=ctk.CTkFont(size=10, weight="bold"))
        self.add_friend_label.grid(row=1, column=1, pady=3, padx=3)
        self.add_friend_entry = ctk.CTkEntry(self, placeholder_text="Username")
        self.add_friend_entry.grid(row=2, column=1, pady=3, padx=3)
        self.add_friend_button = ctk.CTkButton(self, command=self.add_friend_button_press, text="Add")
        self.add_friend_button.grid(row=2, column=2, pady=3, padx=3)
        self.weight_label = ctk.CTkLabel(self, text="Package Weight", font=ctk.CTkFont(size=10, weight="bold"))
        self.weight_label.grid(row=3, column=0, pady=3, padx=3)
        self.dimentions_label = ctk.CTkLabel(self, text="Package Dimentions", font=ctk.CTkFont(size=10, weight="bold"))
        self.dimentions_label.grid(row=3, column=1, pady=3, padx=3)
        self.tracking_label = ctk.CTkLabel(self, text="Order Tracking", font=ctk.CTkFont(size=10, weight="bold"))
        self.tracking_label.grid(row=3, column=2, pady=3, padx=3)
        self.weight_entry = ctk.CTkEntry(self, placeholder_text="kg")
        self.weight_entry.grid(row=4, column=0, pady=3, padx=3)
        self.dimensions_option = ctk.CTkOptionMenu(self, values=["5x5x5cm", "10x10x10cm", "15x15x15"])
        self.dimensions_option.grid(row=4, column=1, pady=3, padx=3)
        self.tracking_switch = ctk.CTkSwitch(self, onvalue="Enabled", offvalue="Disabled", text="Disabled", command=self.tracking_switch_press)
        self.tracking_switch.grid(row=4, column=2, pady=3, padx=3)
        self.order_button = ctk.CTkButton(self, command=self.order_button_press, text="Order", width=450)
        self.order_button.grid(row=5, column=0, columnspan=3, pady=(20, 20), padx=3)

       
    def add_friend_button_press(self):
        self.friend = self.add_friend_entry.get()
        check = self.obj.check_add_friend(self.friend)
        if check == True:
            self.obj.add_friend(self.username, self.friend)
            friend_list = self.obj.fetch_friends(self.username)
            self.update_friend_option_menu(friend_list)
        else:
            print("Username Does Not Exist")

    def update_friend_option_menu(self, friend_list):
        friend_list = self.obj.fetch_friends(self.username)
        options = friend_list["friend"]
        self.dest_username_option.grid_forget()
        self.dest_username_option = ctk.CTkOptionMenu(self, values=options)
        self.dest_username_option.grid(row=2, column=0, pady=3, padx=3)

    def tracking_switch_press(self):
        tracking_val = self.tracking_switch.get()
        self.tracking_switch.configure(text=tracking_val)

    def order_button_press(self):
        self.destination_username = self.dest_username_option.get()
        self.package_weight = self.weight_entry.get()
        self.package_dimentions = self.dimensions_option.get()
        self.tracking_choice = self.tracking_switch.get()
        self.title_logo.grid_forget()
        self.dest_username_label.grid_forget()
        self.dest_username_option.grid_forget()
        self.add_friend_label.grid_forget()
        self.add_friend_entry.grid_forget()
        self.add_friend_button.grid_forget()
        self.weight_label.grid_forget()
        self.dimentions_label.grid_forget()
        self.tracking_label.grid_forget()
        self.weight_entry.grid_forget()
        self.dimensions_option.grid_forget()
        self.tracking_switch.grid_forget()
        self.order_button.grid_forget()
        self.sending_postcode, self.destination_postcode = self.obj.save_order_details(self.username, self.destination_username, self.package_weight, self.package_dimentions, self.tracking_choice)
        #calculate full travel distance (station to sender, sender to reciver, back to station) include weight into calculation
        self.drone_ordered_window()



    def drone_ordered_window(self):
        self.geometry(f"{405}x{300}")
        self.title("Signed in as: " + self.username)
        self.grid_columnconfigure((0, 1, 2), weight=1)

        
        self.title_logo = ctk.CTkLabel(self, text="ORDER IN PROGRESS", font=ctk.CTkFont(size=20, weight="bold"))
        self.title_logo.grid(row=0, column=1, pady=3, padx=3)
        self.progess_bar = ctk.CTkProgressBar(self, width=400, height=10, orientation="horizontal",)
        self.progess_bar.grid(row=1, column=0, columnspan=3, pady=(25, 0), padx=20)
        self.progess_bar.set(0)
        self.status_label = ctk.CTkLabel(self, text="", anchor="center", font=ctk.CTkFont(size=10, weight="bold"))
        self.status_label.grid(row=1, column=1, pady=3, padx=3)

        self.geolocator = Nominatim(user_agent="postcode_converter")
        self.sending_postcode_string = self.sending_postcode[0]
        self.destination_postcode_string = self.destination_postcode[0]
       
        sending_location = self.geolocator.geocode(self.sending_postcode)
        self.sending_coords = (sending_location.latitude, sending_location.longitude)
        print("Sending Coordinates:", self.sending_coords)

        #retrieve coordinates for destination postcode
        destination_location = self.geolocator.geocode(self.destination_postcode)
        self.destination_coords = (destination_location.latitude, destination_location.longitude)
        print("Destination Coordinates:", self.destination_coords)
        self.charging_coords = self.obj.find_nearest_charging_station(self.sending_coords)
        self.energy_required = self.calculate_battery(self.charging_coords, self.sending_coords, self.destination_coords)
        
        package_name = "flymate"
        launch_file = "flymate.launch"

        command = ["roslaunch", package_name, launch_file]
        subprocess.run(command)
        
        
        charging_lat, charging_long = self.charging_coords
        sending_lat, sending_long = self.sending_coords
        delivery_lat, delivery_long = self.destination_coords
        rospy.init_node('param_sender_node')
        rospy.loginfo('param_sender_node started')
        
        rospy.set_param('battery_required', self.energy_required)
        
        rospy.set_param('current_lat', charging_lat)
        rospy.set_param('current_long', charging_long)
        
        rospy.set_param('sending_lat', sending_lat)
        rospy.set_param('sending_long', sending_long)
        
        rospy.set_param('delivery_lat', delivery_lat)
        rospy.set_param('delivery_long', delivery_long)
   
    
        val1 = rospy.get_param('battery_required')
        val2 = rospy.get_param('current_lat')
        val3 = rospy.get_param('current_long')
    
        print("battery required " + str(val1))
        print("current lat " + str(val2))
        print("current long " + str(val3))

        
    def calculate_battery(self, charging_coords, sending_coords, destination_coords):
        energy_per_distance = 0.1 #energy consumed per unit distance
        energy_per_kg = 0.05 #energy consumed per unit weight
        safety_margin = 0.1 #safety margin
        print(self.package_weight)
        
        charging_to_sender_distance = self.obj.calculate_distance(*charging_coords, *sending_coords) #distance in km between coords
        sender_to_reciever_distance = self.obj.calculate_distance(*sending_coords, *destination_coords)
        reciever_to_charging_distance = self.obj.calculate_distance(*destination_coords, *charging_coords)

        print(charging_to_sender_distance)
        print(sender_to_reciever_distance)
        print(reciever_to_charging_distance)

        self.package_weight = int(self.package_weight)
        energy_consumption = energy_per_distance * sender_to_reciever_distance
        energy_consumed_due_to_weight = energy_per_kg * self.package_weight * sender_to_reciever_distance
        total_energy_consumption = energy_consumption + energy_consumed_due_to_weight
        energy_required = total_energy_consumption * (1 + safety_margin)

        print(energy_required)
        return energy_required
    
    def get_params(self):
        return self.energy_required, self.charging_coords, self.sending_coords, self.destination_coords
    
if __name__ == '__main__':
    app = App()
    app.mainloop()
