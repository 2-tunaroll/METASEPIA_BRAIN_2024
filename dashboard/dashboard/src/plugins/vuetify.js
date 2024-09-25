import Vue from 'vue';
import Vuetify from 'vuetify/lib/framework';

Vue.use(Vuetify); 

const darkTheme = {
    dark: true,
    colors: {
        background: '#272727',
        primary: '#FF7C0A',
        secondary: '#0A8DFF'
    }
}
  
export default new Vuetify({
    dark: true,
    icons: {
        iconfont: 'md'
    },
    theme : {
        themes : {
            dark : {
                background: '#272727',
                primary: '#FF7C0A',
                secondary: '#0A8DFF'
            }
        }
    }
});