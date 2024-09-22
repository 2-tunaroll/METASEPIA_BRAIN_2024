<template>
  <img alt="Vue logo" src="./assets/logo.png">
  <p>sub msg: {{ msg }} </p> <!-- 追加 -->
  <div>
    <img v-bind:src="'data:image/jpeg;base64,'+imageBytes" /> 
  </div>
</template>

<script>
import ROSLIB from "roslib"

const ros = new ROSLIB.Ros({ 
  url: "ws://localhost:9090" 
});

const sub_test = new ROSLIB.Topic({
  ros: ros,
  name: "/test",
  messageType: "std_msgs/String"
});

const camera_subscription = new ROSLIB.Topic({
  ros: ros,
  name: "/camera/camera/color/image_raw/compressed",
  messageType: "sensor_msgs/msg/CompressedImage"
})

// const camera_subscription = new ROSLIB.Topic({
//   ros: ros,
//   name: "/camera/camera/depth/image_rect_raw/compressed",
//   messageType: "sensor_msgs/msg/CompressedImage"
// })


export default {
  name: 'App',
  components: {
  },
  // *** 追加 ここから *** //
  data: function () {
    return {
      msg: "",
    };
  },
  // *** 追加 ここまで *** //
  mounted() {
    this.init();
  },
  methods: {
    init: function () {
      
      console.log("test");
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

      sub_test.subscribe((message) => {
        console.log('Received message on ' + sub_test.name + ': ' + message.data);
        self.msg = message.data;
      });

      camera_subscription.subscribe((message) => {
        self.imageBytes = message.data;
      });
    },
  }
}
</script>

<style>
#app {
  font-family: Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}
</style>
