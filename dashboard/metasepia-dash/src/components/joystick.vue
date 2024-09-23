<script setup>
import Joystick from 'vue-joystick-component'

const start = () => console.log('start')
const stop = () => console.log('stop')
const move = ({ x, y, direction, distance }) => {
  console.log('move', { x, y, direction, distance })
}
</script>

<template>
  <Joystick
    :size="100"
    base-color="pink"
    stick-color="purple"
    :throttle="100"
    @start="start"
    @stop="stop"
    @move="move"
  />
</template>
