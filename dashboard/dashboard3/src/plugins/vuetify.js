/**
 * plugins/vuetify.js
 *
 * Framework documentation: https://vuetifyjs.com`
 */

// Styles
import '@mdi/font/css/materialdesignicons.css'
import 'vuetify/styles'

// Composables
import { createVuetify } from 'vuetify'

const darkTheme = {
  dark: true,
  colors: {
      background: '#272727',
      primary: '#FF7C0A',
      secondary: '#0A8DFF'
  }
}

// https://vuetifyjs.com/en/introduction/why-vuetify/#feature-guides
export default createVuetify({
  theme: {
    defaultTheme: 'darkTheme',
    themes: {
      darkTheme
    }
  },
})
