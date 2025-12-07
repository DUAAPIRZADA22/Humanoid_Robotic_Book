/**
 * Creating a sidebar enables you to:
 - create an ordered group of docs
 - render a sidebar for each docs group
 - provide next/previous navigation

 The sidebars can be generated from the filesystem, or explicitly defined here.

 Create as many sidebars as you want.
 */

/** @type {import('@docusaurus/plugin-content-docs').SidebarsConfig} */
const sidebars = {
  // By default, Docusaurus generates a sidebar from the docs folder structure
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Course Content',
      items: [
        'intro',
        {
          type: 'category',
          label: 'Module 1: Foundations',
          items: [
            'module-1/foundations',
            'module-1/setup',
            'module-1/kinematics',
          ],
        },
        {
          type: 'category',
          label: 'Module 2: Physical AI & Humanoid Robotics',
          items: [
            {
              type: 'category',
              label: 'Part 1: Foundations (Weeks 1-2)',
              items: [
                'module-2/part1-foundations/chapter-1-introduction',
                'module-2/part1-foundations/chapter-2-sensors-perception',
              ],
            },
            {
              type: 'category',
              label: 'Part 2: The Nervous System (Weeks 3-5)',
              items: [
                'module-2/part2-nervous-system/chapter-3-ros2-architecture',
                'module-2/part2-nervous-system/chapter-4-building-ros2-nodes-python',
                'module-2/part2-nervous-system/chapter-5-launch-systems-parameter-management',
              ],
            },
            {
              type: 'category',
              label: 'Part 3: The Digital Twin (Weeks 6-7)',
              items: [
                'module-2/part3-digital-twin/chapter-6-gazebo-simulation',
                'module-2/part3-digital-twin/chapter-7-physics-simulation-unity',
              ],
            },
            {
              type: 'category',
              label: 'Part 4: The AI Brain',
              items: [
                'module-2/part4-ai-brain/chapter-8-nvidia-isaac-sim',
              ],
            },
            // {
            //   type: 'category',
            //   label: 'Part 5: Advanced Humanoids',
            //   items: [
            //     'module-2/part5-advanced-humanoids/chapter-11-humanoid-kinematics',
            //     'module-2/part5-advanced-humanoids/chapter-12-vla-conversational',
            //   ],
            // },
          ],
        },
        {
          type: 'category',
          label: 'Module 3: Control',
          items: [
            'module-3/control',
          ],
        },
        {
          type: 'category',
          label: 'Module 4: Applications',
          items: [
            'module-4/applications',
          ],
        },
      ],
    },
  ],
  // But you can create a sidebar manually
  /*
  tutorialSidebar: [
    {
      type: 'category',
      label: 'Tutorial',
      items: ['hello'],
    },
  ],
   */
};

module.exports = sidebars;