<template>
    <v-container>
      <!-- video stream -->
      <v-row >
        <v-col>
          <v-container>
            <img v-bind:src="'data:image/jpeg;base64,' + imageBytes1"
            width="700"
            cover
            />
          </v-container>
        </v-col>

        <v-col>
          <v-card>
            <v-container>
              <p>Voltage : {{ Math.round(voltage * 100) / 100 }}</p>
              <p>Charge  : {{ Math.floor(charge) }}%</p>
              <v-progress-linear 
                v-model="charge"
                color="primary"
              ></v-progress-linear>
            </v-container>
          </v-card>
          
          <v-card>
            <v-container>
              <p>surge</p>
              <v-progress-linear 
                model-value="100"
              ></v-progress-linear>
            </v-container>
            <v-container>
              <p>sway</p>
              <v-progress-linear 
                model-value="100"
              ></v-progress-linear>
            </v-container>
            <v-container>
              <p>pitch</p>
              <v-progress-linear 
                model-value="100"
              ></v-progress-linear>
            </v-container>
            <v-container>
              <p>yaw</p>
              <v-progress-linear 
                model-value="100"
              ></v-progress-linear>
            </v-container>
          </v-card>
        </v-col>
      </v-row>
    </v-container>
</template>

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
        charge: 0,
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
        self.charge = message.charge;
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
  }
}
</script>
