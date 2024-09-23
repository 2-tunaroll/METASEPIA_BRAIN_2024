import Vue from 'vue';
import Vuetify from 'vuetify/lib/framework';

Vue.use(Vuetify);

const dark_theme = {
    dark: true,
    colors: {
        background: '#272727',
        primary: '#FF7C0A',
        secondary: '#0A8DFF'
    },
    variables: {
    }
}

export default new Vuetify({
    theme: {
        defaultTheme: 'dark_theme',
        themes: {
            dark_theme
        },
    },
});
