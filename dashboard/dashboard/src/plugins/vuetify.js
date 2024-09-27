import Vue from 'vue'
import Vuetify from 'vuetify'
import 'vuetify/dist/vuetify.min.css'

Vue.use(Vuetify)


const darkTheme = {
    dark: true,
    colors: {
        background: '#272727',
        primary: '#FF7C0A',
        secondary: '#0A8DFF'
    }
}

export default new Vuetify({
    theme: {
        defaultTheme: 'darkTheme',
        themes: {
            darkTheme
        }
    }
})

  