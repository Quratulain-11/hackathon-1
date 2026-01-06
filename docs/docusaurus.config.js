// @ts-check
// `@type` JSDoc annotations allow IDEs and type-checking tools to autocomplete
// and validate function arguments and return types.
// You don't need to do this with your config, but it's a good practice.

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Embodied Intelligence - Connecting AI Cognition with Real-World Robotic Action',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://hackathon-1-o3om.vercel.app', // Vercel deployment URL
  // Set the /<baseUrl>/ pathname under which your site is served
  // For deployment in docs subdirectory served from root: use '/'
  baseUrl: '/',

  // Vercel deployment configuration
  organizationName: 'Quratulain-11', // GitHub username for Vercel deployment
  projectName: 'hackathon-1', // GitHub repository name
  deploymentBranch: 'main', // Branch that Vercel deploys from
  trailingSlash: undefined, // Let Docusaurus handle trailing slashes for Vercel

  onBrokenLinks: 'warn',
  onBrokenMarkdownLinks: 'warn',

  // Even if you don't use internationalization, you can use this field to set
  // useful metadata like html lang. For example, if your site is Chinese, you
  // may want to replace "en" with "zh-Hans".
  i18n: {
    defaultLocale: 'en',
    locales: ['en'],
  },

  presets: [
    [
      'classic',
      /** @type {import('@docusaurus/preset-classic').Options} */
      ({
        docs: {
          sidebarPath: require.resolve('./sidebars.js'),
          // Exclude node_modules from being processed as docs
          exclude: ['node_modules/**'],
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/Quratulain-11/hackathon-1/edit/main/docs/',
        },
        blog: false, // Disable blog if not needed
        theme: {
          customCss: require.resolve('./src/css/custom.css'),
        },
      }),
    ],
  ],

  themeConfig:
    /** @type {import('@docusaurus/preset-classic').ThemeConfig} */
    ({
      // Replace with your project's social card
      image: 'img/docusaurus-social-card.jpg',
      navbar: {
        title: 'Physical AI & Humanoid Robotics',
        logo: {
          alt: 'Physical AI Logo',
          src: 'img/logo.svg',
        },
        items: [
          {
            type: 'docSidebar',
            sidebarId: 'tutorialSidebar',
            position: 'left',
            label: 'Book',
          },
          {
            href: 'https://github.com/Quratulain-11/hackathon-1',
            label: 'GitHub',
            position: 'right',
          },
        ],
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Book Modules',
            items: [
              {
                label: 'Module 1: ROS 2 Nervous System',
                to: '/docs/module-1-ros-nervous-system/',
              },
              {
                label: 'Module 2: Digital Twin',
                to: '/docs/module-2-digital-twin/',
              },
              {
                label: 'Module 3: AI-Robot Brain',
                to: '/docs/module-3-ai-robot-brain/',
              },
              {
                label: 'Module 4: Vision-Language-Action',
                to: '/docs/module-4-vision-language-action/',
              },
            ],
          },
          {
            title: 'Community',
            items: [
              {
                label: 'Stack Overflow',
                href: 'https://stackoverflow.com/questions/tagged/docusaurus',
              },
              {
                label: 'Discord',
                href: 'https://discordapp.com/invite/docusaurus',
              },
              {
                label: 'Twitter',
                href: 'https://twitter.com/docusaurus',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/Quratulain-11/hackathon-1',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics Book. Built with Docusaurus.`,
      },
      prism: {
        theme: require('prism-react-renderer').themes.github,
        darkTheme: require('prism-react-renderer').themes.dracula,
      },
    }),
};

module.exports = config;