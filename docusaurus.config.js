// @ts-check
// Note: type annotations allow type checking and IDEs autocompletion

const lightCodeTheme = require('prism-react-renderer').themes.github;
const darkCodeTheme = require('prism-react-renderer').themes.vsDark;

/** @type {import('@docusaurus/types').Config} */
const config = {
  title: 'Physical AI & Humanoid Robotics',
  tagline: 'Master embodied intelligence through comprehensive AI-driven robotics courses',
  favicon: 'img/favicon.ico',

  // Set the production url of your site here
  url: 'https://humanoid-robotic-book.vercel.app',
  // Set the /<baseUrl>/ pathname under which your site is served
  // For Vercel deployment, use root path
  baseUrl: '/',

  // Vercel deployment config.
  organizationName: 'DUAAPIRZADA22', // Your GitHub org/user name.
  projectName: 'Humanoid_Robotic_Book', // Your repo name.
  deploymentBranch: 'main',

  onBrokenLinks: 'throw',
  markdown: {
    hooks: {
      onBrokenMarkdownLinks: 'warn'
    }
  },

  // Even if you don't use internalization, you can use this field to set useful
  // metadata like html lang. For example, if your site is Chinese, you may want
  // to replace "en" with "zh-Hans".
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
          sidebarPath: './sidebars.js',
          // Please change this to your repo.
          // Remove this to remove the "edit this page" links.
          editUrl:
            'https://github.com/DUAAPIRZADA22/Humanoid_Robotic_Book/tree/main/',
        },
          theme: {
          customCss: './src/css/custom.css',
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
            type: 'doc',
            docId: 'intro',
            position: 'left',
            label: 'Course',
          },
          {
            to: '/foundation-ai-integration',
            position: 'left',
            label: 'AI Integration',
          },
          {
            href: 'https://github.com/DUAAPIRZADA22/Humanoid_Robotic_Book',
            label: 'GitHub',
            position: 'right',
          },
        ],
        hideOnScroll: false,
      },
      footer: {
        style: 'dark',
        links: [
          {
            title: 'Course',
            items: [
              {
                label: 'Introduction',
                to: '/docs/intro',
              },
              {
                label: 'Module 1: Foundations',
                to: '/docs/module-1/foundations',
              },
              {
                label: 'AI Integration',
                to: '/foundation-ai-integration',
              },
            ], 





            
          },
          {
            title: 'Community',
            items: [
              {
                label: 'GitHub',
                href: 'https://github.com/DUAAPIRZADA22/Humanoid_Robotic_Book',
              },
            ],
          },
          {
            title: 'More',
            items: [
              {
                label: 'Setup Guide',
                to: '/docs/module-1/setup',
              },
              {
                href: 'https://github.com/facebook/docusaurus',
                label: 'Docusaurus',
              },
            ],
          },
        ],
        copyright: `Copyright © ${new Date().getFullYear()} Physical AI & Humanoid Robotics. Built with Docusaurus.`,
      },
      prism: {
        theme: lightCodeTheme,
        darkTheme: darkCodeTheme,
        additionalLanguages: ['python', 'javascript', 'typescript', 'bash', 'json'],
      },
      // Default theme configuration
      colorMode: {
        defaultMode: 'light',
        disableSwitch: false,
        respectPrefersColorScheme: false,
      },
    }),

  // Custom fields for your site
  customFields: {
    // Backend API URL - use NEXT_PUBLIC_API_URL env var, fallback to Railway/localhost
    chatApiEndpoint: process.env.NEXT_PUBLIC_API_URL ||
      (process.env.NODE_ENV === 'production'
        ? 'https://humanoidroboticbook-production-ccff.up.railway.app'
        : 'http://localhost:8000'),
    chatApiKey: process.env.NEXT_PUBLIC_API_KEY || '',  // Optional - backend doesn't require API key
    // Translation service API URL (same as backend)
    translateApiUrl: process.env.NEXT_PUBLIC_API_URL ||
      (process.env.NODE_ENV === 'production'
        ? 'https://humanoidroboticbook-production-ccff.up.railway.app'
        : 'http://localhost:8000'),
  },
};

module.exports = config;