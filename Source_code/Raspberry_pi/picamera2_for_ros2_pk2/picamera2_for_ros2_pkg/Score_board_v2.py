import tkinter as tk
import rclpy
from rclpy import init, shutdown
from rclpy.node import Node
from std_msgs.msg import String
import time
from threading import Thread


class ScoreboardSubscriber(Node):
    def __init__(self,scoreboard_app_instance):
        super().__init__("scoreboard_subscriber")
        self.scoreboard_subscriber = self.create_subscription(String, '/scoreboard_topic', self.scoreboard_callback, 10)
        self.scoreboard_subscriber  # prevent unused variable warning
        self.scoreboard_app_instance = scoreboard_app_instance
        time.sleep(1)  # give the connection a second to settle

    def scoreboard_callback(self, msg):
        self.get_logger().info(str(msg))
        team = int(float(msg.data[0]))
        print("hello_from_callback ")
        print(team)
        self.scoreboard_app_instance.increase_score(team)


class Send_msg_motor(Node):
    def __init__(self):
        super().__init__("scoreboard_start")
        self.publisher = self.create_publisher(String, 'motor_topic',10 )
    
    def publish_message(self,message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)

def Thread_app_subscribe(scoreboard_app_instance):
    scoreboardsubscriber = ScoreboardSubscriber(scoreboard_app_instance)
    rclpy.spin(scoreboardsubscriber)
    scoreboardsubscriber.destroy_node()

class ScoreboardApp(tk.Tk):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.title("Scoreboard")

        # Blue background
        self.configure(background='blue')

        self.team1_score = 0
        self.team2_score = 0

        # Team 1 score label on left
        self.team1_label = tk.Label(self, text="Team 1 Score: 0", font=("Arial", 24), fg='white', bg='blue')
        self.team1_label.pack(pady=5, side=tk.LEFT)

        # Team 2 score label on right
        self.team2_label = tk.Label(self, text="Team 2 Score: 0", font=("Arial", 24), fg='white', bg='blue')
        self.team2_label.pack(pady=5, side=tk.RIGHT)

        self.time_label = tk.Label(self, text="Time: 0:00", font=("Arial", 24), fg='white', bg='blue')
        self.time_label.pack(pady=5)

        # Start button to begin the timer
        self.start_button = tk.Button(self, text="Start", command=self.start_timer)
        self.start_button.pack(pady=5)

        self.timer_running = False
        self.start_time = None
        self.check_start = False

    def start_timer(self):
        if self.check_start == False: 
            Send_msg_motor().publish_message('S')
            self.check_start = True
        if not self.timer_running:
            self.start_time = time.time()
            self.update_time()
            self.start_button.config(text="Pause")
        else:
            self.start_button.config(text="Resume")
        self.timer_running = not self.timer_running

    def update_time(self):
        if self.timer_running:
            elapsed_time = int(time.time() - self.start_time)
            minutes = elapsed_time // 60
            seconds = elapsed_time % 60
            time_str = f"Time: {minutes}:{seconds:02d}"
            self.time_label.config(text=time_str)
        self.after(1000, self.update_time)

    def increase_score(self, team):

        if team == 1:
            self.team1_score += 1
            self.team1_label.config(text=f"Team 1 Score: {self.team1_score}")
        elif team == 2:
            self.team2_score += 1
            self.team2_label.config(text=f"Team 2 Score: {self.team2_score}")
        elif team == 3:
            pass

    def decrease_score(self, team):
        if team == 1:
            if self.team1_score > 0:
                self.team1_score -= 1
                self.team1_label.config(text=f"Team 1 Score: {self.team1_score}")
        elif team == 2:
            if self.team2_score > 0:
                self.team2_score -= 1
                self.team2_label.config(text=f"Team 2 Score: {self.team2_score}")


def main(args=None):
    rclpy.init(args=args)
    # scoreboardsubscriber = ScoreboardSubscriber()
    motor_publisher = Send_msg_motor()
    # rclpy.spin(scoreboardsubscriber)
    root = ScoreboardApp()
    subscriber_instance = ScoreboardSubscriber(root)
    thread = Thread(target= Thread_app_subscribe, args = (root,))
    thread.start()
    root.mainloop()
    # scoreboardsubscriber.destroy_node()
    motor_publisher.destroy_node()
    
    rclpy.shutdown()


if __name__ == '__main__':
    main()