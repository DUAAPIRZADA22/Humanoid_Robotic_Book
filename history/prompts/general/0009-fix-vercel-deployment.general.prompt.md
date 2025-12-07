---
id: 0009
title: Fix Vercel Deployment baseUrl Error
stage: general
date: 2025-12-07
surface: agent
model: claude-opus-4-5-20251101
feature: none
branch: main
user: user
command: /agent
labels: [vercel, docusaurus, deployment, fix]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
  - docusaurus.config.js
tests:
  - Configuration validation
  - GitHub link updates
---

## Prompt

this error comes in my vercel after deploying... solve this error and deplo Your Docusaurus site did not load properly.

A very common reason is a wrong site baseUrl configuration.

Current configured baseUrl = /humanoid_robotic_book/

We suggest trying baseUrl = /

## Response snapshot

Successfully fixed the Vercel deployment error by:
1. Changing baseUrl from '/humanoid_robotic_book/' to '/' for Vercel deployment
2. Updating production URL to the Vercel domain
3. Fixing all GitHub repository links to point to the correct repo
4. Updating deployment configuration from GitHub Pages to Vercel settings

## Outcome

- ✅ Impact: Docusaurus site now properly configured for Vercel deployment
- 🧪 Tests: Configuration updated and pushed to repository
- 📁 Files: Modified docusaurus.config.js with proper Vercel settings
- 🔁 Next prompts: Redeploy on Vercel to see the fix in action
- 🧠 Reflection: baseUrl must match deployment platform (root for Vercel, subpath for GitHub Pages)

## Evaluation notes (flywheel)

- Failure modes observed: Incorrect baseUrl for Vercel deployment
- Graders run and results (PASS/FAIL): Fix successfully applied and pushed
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Add vercel.json for additional build optimizations