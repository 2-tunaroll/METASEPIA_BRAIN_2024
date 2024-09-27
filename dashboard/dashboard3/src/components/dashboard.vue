<template>
    <v-container>    
      <v-row no-gutters justify="space between">
        <v-col>
            <v-container>
                <img v-bind:src="'data:image/jpeg;base64,' + imageBytes1"
                cover 
                style="border-radius: 2%;"
                width="650"
                />
            </v-container>
        </v-col> 
        <v-col justify="center">
          <v-row no-gutters>
            <v-container>
                <v-card
                >
                    <template v-slot:text>
                        <pre>
voltage : {{ Math.round(average_voltage * 100) / 100 }}
charge  : {{ Math.floor(average_charge) }}%
                        </pre>
                        <v-progress-linear 
                            v-model="average_charge"
                            color="primary"
                            rounded="pill"
                        ></v-progress-linear>
                    </template>
                </v-card>
            </v-container>
            
          </v-row>
          <v-row no-gutters>
            <v-container>
                <v-card
                >
                    <template v-slot:text>
                        <pre>
surge   : {{Math.round(surge * 100) / 100}} 
sway    : {{Math.round(sway * 100) / 100}}
pitch   : {{ Math.round(pitch * 100) / 100 }}
yaw     : {{ Math.round(yaw * 100) / 100 }}
                        </pre>
                        <v-row justify="end">
                            <div style="padding-right: 30px; padding-left: 30px; padding-bottom: 30px; padding-top: 30px">
                                <Joystick
                                :size="75"
                                base-color="#494949"
                                stick-color="#FF7C0A"
                                :throttle="75"
                                @move="move"
                                />
                            </div>
                            <div style="padding-right: 30px; padding-left: 30px; padding-bottom: 30px; padding-top: 30px">
                                <Joystick
                                :size="75"
                                base-color="#494949"
                                stick-color="#FF7C0A"
                                :throttle="75"
                                @move="move"
                                />
                            </div> 
                        </v-row>
                    </template>
                </v-card>
            </v-container>
          </v-row>
        </v-col>
      </v-row>  
  </v-container>
</template>

<script setup>
    import Joystick from 'vue-joystick-component'
    const start = () => console.log('start')
    const stop = () => console.log('stop')
    const move = ({ x, y, direction, distance }) => {
        x = surge;
        console.log('move', { x, y, direction, distance });
    }
</script>

<script>
import ROSLIB from "roslib"

const ros = new ROSLIB.Ros({ 
  url: "ws://localhost:9090" 
});

const battery_sub = new ROSLIB.Topic({
  ros: ros,
  name: "/BatteryState",
  messageType: "custom_msgs/BatteryState"
});

const camera_sub = new ROSLIB.Topic({
  ros: ros,
  name: "/camera/camera/color/image_raw/compressed",
  messageType: "sensor_msgs/msg/CompressedImage"
})

const cmd_sub = new ROSLIB.Topic({
  ros: ros,
  name: "/CMD",
  messageType: "custom_msgs/CMD"
})

export default {
  name: 'App',
  components: {
  },
  data: function () {
    return {
      imageBytes1: "",
      voltage: 0,
      voltage_window: [],
      average_voltage: 0,
      charge: 0,
      charge_window: [],
      average_charge: 0,
      surge: 0,
      sway: 0,
      pitch: 0,
      yaw: 0
    };
  },
  mounted() {
    this.init();
  },
  methods: {
    init: function () {
      ros.on("connection", function() {
        console.log("connected to websocket server.");
      });
      ros.on("error", function(error) {
        console.log("error connecting to websocket server: ", error);
      });
      ros.on("close", function() {
        console.log("connection to websocket server closed.");
      });

      var self = this;

      battery_sub.subscribe((message) => {
        console.log('Received message on ' + battery_sub.name + ': ' + message.voltage + ': ' + message.charge);
        self.voltage = message.voltage;
        self.add_voltage(message.voltage);

        self.charge = message.charge;
        self.add_charge(message.charge);
      });

      camera_sub.subscribe((message) => {
        console.log('Received message on ' + camera_sub.name);
        self.imageBytes1 = message.data;
      });

      cmd_sub.subscribe((message) => {
        console.log('Received message on ' + cmd_sub.name + ': ' + message.surge + ': ' + message.sway + ': ' + message.pitch + ': ' + message.yaw);
        self.surge = message.surge;
        self.sway = message.sway;
        self.pitch = message.pitch;
        self.yaw = message.yaw;
      });

    },
    add_voltage(volatge) {
      if (this.voltage_window.length >= 50) {
        this.voltage_window.shift();
      }
      this.voltage_window.push(volatge);
      this.calculate_average_voltage();
    },
    calculate_average_voltage() {
      if (this.voltage_window.length > 0) {
        this.average_voltage = this.voltage_window.reduce((partialSum, a) => partialSum + a, 0) / this.voltage_window.length;
      }
    },
    add_charge(charge) {
      if (this.charge_window.length >= 100) {
        this.charge_window.shift();
      }
      this.charge_window.push(charge);
      this.calculate_average_charge();
    },
    calculate_average_charge() {
      if (this.charge_window.length > 0) {
        this.average_charge = this.charge_window.reduce((partialSum, a) => partialSum + a, 0) / this.charge_window.length;
      }
    }
  }
}
</script>
