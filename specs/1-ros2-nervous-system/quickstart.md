# Quickstart: Physical AI & Humanoid Robotics - Module 1: The Robotic Nervous System (ROS 2)

## Prerequisites

- Node.js (v16 or higher)
- npm or yarn package manager
- Git
- Basic knowledge of Python (for understanding rclpy concepts)
- Understanding of AI fundamentals

## Setup Docusaurus Project

1. **Initialize Docusaurus**:
   ```bash
   npx create-docusaurus@latest website classic
   ```

2. **Navigate to project directory**:
   ```bash
   cd website
   ```

3. **Install additional dependencies** (if needed):
   ```bash
   npm install
   ```

## Configure Docusaurus for the Module

1. **Create module directory structure**:
   ```bash
   mkdir -p docs/modules/physical-ai-robotics/module-1-ros2-nervous-system
   ```

2. **Update sidebar configuration** in `sidebars.js`:
   ```javascript
   module.exports = {
     tutorialSidebar: [{
       type: 'category',
       label: 'Physical AI & Humanoid Robotics',
       items: [{
         type: 'category',
         label: 'Module 1: The Robotic Nervous System (ROS 2)',
         items: [
           'modules/physical-ai-robotics/module-1-ros2-nervous-system/chapter-1-ros2-fundamentals',
           'modules/physical-ai-robotics/module-1-ros2-nervous-system/chapter-2-ai-agents-ros2',
           'modules/physical-ai-robotics/module-1-ros2-nervous-system/chapter-3-robot-anatomy-urdf'
         ],
       }],
     }],
   };
   ```

3. **Create the three chapter files**:
   - `docs/modules/physical-ai-robotics/module-1-ros2-nervous-system/chapter-1-ros2-fundamentals.md`
   - `docs/modules/physical-ai-robotics/module-1-ros2-nervous-system/chapter-2-ai-agents-ros2.md`
   - `docs/modules/physical-ai-robotics/module-1-ros2-nervous-system/chapter-3-robot-anatomy-urdf.md`

## Authoring Content

1. **Start with Chapter 1**: Begin with ROS 2 fundamentals, explaining the middleware concept, nodes, topics, services, and messages.

2. **Continue with Chapter 2**: Cover connecting AI agents to robots using rclpy, focusing on the perception→decision→actuation flow.

3. **Complete with Chapter 3**: Explain URDF for robot anatomy, covering links, joints, sensors, and how URDF enables simulation.

## Local Development

1. **Start development server**:
   ```bash
   npm run start
   ```

2. **Build for production**:
   ```bash
   npm run build
   ```

3. **Deploy to GitHub Pages** (after configuration):
   ```bash
   npm run deploy
   ```

## GitHub Pages Configuration

1. **Update `docusaurus.config.js`** with GitHub Pages settings:
   ```javascript
   module.exports = {
     // ...
     url: 'https://your-username.github.io',
     baseUrl: '/your-repo-name/',
     organizationName: 'your-username',
     projectName: 'your-repo-name',
     deploymentBranch: 'gh-pages',
     // ...
   };
   ```

## Validation Steps

1. **Verify all links work**: Test navigation between chapters
2. **Check content formatting**: Ensure MDX renders correctly
3. **Validate build**: Confirm `npm run build` completes without errors
4. **Review content accuracy**: Verify technical content is correct and appropriate for the target audience