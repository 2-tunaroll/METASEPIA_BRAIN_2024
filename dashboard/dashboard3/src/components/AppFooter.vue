<template>
  <v-footer height="40" app>
    <a
      v-for="item in items"
      :key="item.title"
      :href="item.href"
      :title="item.title"
      class="d-inline-block mx-2 social-link"
      rel="noopener noreferrer"
      target="_blank"
    >
      <v-icon
        :icon="item.icon"
        :size="item.icon === '$vuetify' ? 24 : 16"
      />
    </a>

    <div
      class="text-caption text-disabled"
      style="position: absolute; right: 16px;"
    >
      <span class="d-none d-sm-inline-block">The University of Adelaide, 2024</span>
      —
      <a
        class="text-decoration-none on-surface"
        rel="noopener noreferrer"
        target="_blank"
      >
        Metasepia
      </a>
    </div>
  </v-footer>
</template>

<script setup>
  const items = [
    {
      title: 'Metasepia GitHub',
      icon: `mdi-github`,
      href: 'https://github.com/2-tunaroll/METASEPIA_BRAIN_2024',
    },
  ]
</script>

<style scoped lang="sass">
  .social-link :deep(.v-icon)
    color: rgba(var(--v-theme-on-background), var(--v-disabled-opacity))
    text-decoration: none
    transition: .2s ease-in-out

    &:hover
      color: rgba(25, 118, 210, 1)
</style>
