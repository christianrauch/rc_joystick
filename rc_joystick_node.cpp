#include <libevdev/libevdev.h>
#include <fcntl.h>
#include <dirent.h>
#include <unistd.h>
#include <cstring>

#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>

#include <mavros_msgs/msg/override_rc_in.hpp>

#include <chrono>

using namespace std::literals::chrono_literals;

class AxesEventReader : public rclcpp::Node {
private:
    static const std::string event_dir;
    static const std::string event_file;

    static const uint16_t rcmin;
    static const uint16_t rcmax;

    std::string device;

    rclcpp::Publisher<mavros_msgs::msg::OverrideRCIn>::SharedPtr pub_rc;
    rclcpp::TimerBase::SharedPtr timer;

    int fd;
    struct libevdev *dev = NULL;

    std::vector<uint> valid_evcodes;
    std::array<int, ABS_MAX> axmin, axmax;

    mavros_msgs::msg::OverrideRCIn msg;

    std::thread ev_block_thread;
    std::mutex msg_mutex;

public:
    AxesEventReader() :
        Node("rc_joystick"),
        pub_rc(create_publisher<mavros_msgs::msg::OverrideRCIn>("rc/override/raw", 1))
    {
        device = declare_parameter("device", "");

        if(!connect()) {
            RCLCPP_ERROR(get_logger(), "could not connect to any joystick");
            rclcpp::shutdown();
        }

        // start event thread
        start();

        // start main loop
        timer = create_wall_timer(10ms, std::bind(&AxesEventReader::loop, this) );
    }

    ~AxesEventReader() {
        libevdev_free(dev);
        close(fd);
    }

    bool connect() {
        // get list of event devices
        const std::map<std::string, std::string> evdevs = AxesEventReader::getEventList();
        if(evdevs.size()==0) {
            RCLCPP_ERROR(get_logger(), "no devices available");
            return false;
        }

        // read parameters
        if(device.empty()) {
            // use first available device
            device = evdevs.begin()->first;
        }
        else {
            if(evdevs.count(device)==0) {
                // no such evdev device
                RCLCPP_ERROR(get_logger(), "device %s is not an event device", device.c_str());
                std::cout << "Available devices:" << std::endl;
                for(const auto &p : evdevs) {
                    std::cout << p.second << " (" << p.first << ")" << std::endl;
                }
                return false;
            }
        }

        fd = open(device.c_str(), O_RDONLY);
        if(libevdev_new_from_fd(fd, &dev) < 0) {
            RCLCPP_ERROR(get_logger(), "libevdev error");
            close(fd);
            return false;
        }
        RCLCPP_INFO(get_logger(), "device: '%s' (%s)", libevdev_get_name(dev), device.c_str());
        RCLCPP_INFO(get_logger(), "firmware: %i", libevdev_get_id_version(dev));

        // check if device is joystick
        if(!libevdev_has_event_type(dev, EV_ABS)) {
            RCLCPP_ERROR(get_logger(), "device is not a joystick");
            libevdev_free(dev);
            close(fd);
            return false;
        }

        // get minimum and maximum value ranges
        for (uint icode(0); icode < ABS_MAX; icode++) {
            if(libevdev_has_event_code(dev, EV_ABS, icode)) {
                valid_evcodes.push_back(icode);
                const int vmin = libevdev_get_abs_minimum(dev, icode);
                const int vmax = libevdev_get_abs_maximum(dev, icode);
                axmin[icode] = vmin;
                axmax[icode] = vmax;
                RCLCPP_INFO(get_logger(), "%s: min=%i max=%i", libevdev_event_code_get_name(EV_ABS, icode), vmin, vmax);
                if (icode >= msg.channels.max_size()) {
                    RCLCPP_WARN_STREAM(get_logger(), "event code " << icode << " exceeds max channels " << msg.channels.max_size());
                }
            }
        }

        if(axmin.size()!=axmax.size()) {
            RCLCPP_ERROR(get_logger(), "amount of axes do not match");
            return false;
        }

        return true;
    }

    void start() {
        ev_block_thread = std::thread([this](){
            struct input_event ev;
            while(rclcpp::ok()) {
                int rc = libevdev_next_event(dev, LIBEVDEV_READ_FLAG_BLOCKING, &ev);
                if(rclcpp::ok() && rc==LIBEVDEV_READ_STATUS_SUCCESS) {
                    if(ev.type==EV_ABS) {
                        const int32_t axval = ev.value;
                        msg_mutex.lock();
                        // map [min,max] to [1000,2000]
                        msg.channels[ev.code] = rcmin + ((axval-axmin[ev.code])*(rcmax-rcmin)) / (axmax[ev.code]-axmin[ev.code]);
                        pub_rc->publish(msg);
                        msg_mutex.unlock();
                    }
                }
                else if(rc==LIBEVDEV_READ_STATUS_SYNC) {
                    RCLCPP_ERROR(get_logger(), "out of sync");
                }
                else if(rc==-ENODEV){
                    RCLCPP_ERROR(get_logger(), "device disconnected, shutting down");
                    rclcpp::shutdown();
                }
                else {
                    RCLCPP_ERROR(get_logger(), "unknown return value: %i", rc);
                }
            }
        });
        ev_block_thread.detach();
    }

    // periodically check axis events
    void loop() {
        msg_mutex.lock();
        for (const uint &iaxis : valid_evcodes) {
            const int axval = libevdev_get_event_value(dev, EV_ABS, iaxis);
            if (axmax[iaxis] == axmin[iaxis]) {
                msg.channels[iaxis] = axval;
            }
            else {
                msg.channels[iaxis] = rcmin + ((axval-axmin[iaxis])*(rcmax-rcmin)) / (axmax[iaxis]-axmin[iaxis]);
            }
        }
        pub_rc->publish(msg);
        msg_mutex.unlock();
    }

    static
    std::map<std::string, std::string> getEventList() {
        std::map<std::string, std::string> evdevices;

        DIR *dinp = opendir(event_dir.c_str());
        struct dirent *dir;
        while ((dir = readdir(dinp)) != NULL) {
            if(std::strncmp(dir->d_name, event_file.c_str(), event_file.size())!=0)
                continue;
            const std::string evpath = event_dir+"/"+std::string(dir->d_name);
            const int fd = open(evpath.c_str(), O_RDONLY|O_NONBLOCK);
            if(fd<0)
                continue;
            char evdev_name[256];
            ioctl(fd, EVIOCGNAME(sizeof(evdev_name)), evdev_name);
            evdevices[evpath] = std::string(evdev_name);
            close(fd);
        }
        free(dir);
        closedir(dinp);

        return evdevices;
    }
};

const std::string AxesEventReader::event_dir = "/dev/input";
const std::string AxesEventReader::event_file = "event";

const uint16_t AxesEventReader::rcmin = 1000;
const uint16_t AxesEventReader::rcmax = 2000;


int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<AxesEventReader>());
    rclcpp::shutdown();
    return 0;
}
