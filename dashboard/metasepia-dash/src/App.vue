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
        imageBytes2: "",
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

<template>
  <div class="flex min-h-screen w-full bg-gray-700 font-sans">
    <p>voltage : {{ voltage }} </p> <!-- 追加 -->
    <p>charge : {{ charge }} </p>
    <div>
      <img v-bind:src="'data:image/jpeg;base64,'+ imageBytes1 " /> 
    </div>
    <p>surge : {{ surge }} sway : {{ sway }} pitch : {{ pitch }} yaw : {{ yaw }}</p>
  </div>
</template>
