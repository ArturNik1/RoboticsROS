from subprocess import Popen
import time

if __name__ == "__main__":
    p1 = Popen(['rosrun', 'robotics_one', 'draw_letter_server.py'])
    p2 = Popen(["rosrun", "robotics_one", "print_log_info.py"])

    p3 = Popen("""rosrun robotics_one draw_letter_client.py '[1.0, 5.0]' "a" '[255, 0, 0]'""", shell=True)
    p3.wait()
    p4 = Popen("""rosrun robotics_one draw_letter_client.py '[4.0, 7.0]' "z" '[255, 0, 0]'""", shell=True)
    p4.wait()
    p5 = Popen("""rosrun robotics_one draw_letter_client.py '[7.0, 5.0]' "o" '[255, 0, 0]'""", shell=True)
    p5.wait()
    p6 = Popen("""rosrun robotics_one draw_letter_client.py '[9.0, 5.0]' "o" '[255, 0, 0]'""", shell=True)
    p6.wait()
    p7 = Popen(["rosservice", "call", "/print_log_info/print"])

    time.sleep(2)

    p7.terminate()
    p2.terminate()
    p1.terminate()


