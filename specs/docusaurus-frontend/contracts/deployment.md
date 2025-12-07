# Deployment Contract

**Project**: Physical AI & Humanoid Robotics Book
**Module**: 1 - Frontend Foundation
**Version**: 1.0.0
**Date**: 2025-12-04

## GitHub Pages Deployment

### Trigger Conditions
```yaml
on:
  push:
    branches: [ main ]
  pull_request:
    branches: [ main ]
```

### Environment Variables
```yaml
env:
  NODE_VERSION: 18
  CACHE_DEPENDENCY_PATH: |
    ~/.npm
    node_modules
```

### Build Job
```yaml
build:
  name: Build Docusaurus
  runs-on: ubuntu-latest
  steps:
    - name: Checkout
      uses: actions/checkout@v4
      with:
        fetch-depth: 0  # For git history

    - name: Setup Node.js
      uses: actions/setup-node@v4
      with:
        node-version: ${{ env.NODE_VERSION }}
        cache: 'npm'

    - name: Install Dependencies
      run: npm ci

    - name: Validate Configuration
      run: |
        npm run validate-config || true
        npm run check-links || true

    - name: Build Website
      run: npm run build
      env:
        NODE_ENV: production
        GOOGLE_ANALYTICS_ID: ${{ secrets.GOOGLE_ANALYTICS_ID }}

    - name: Upload Build Artifacts
      uses: actions/upload-pages-artifact@v3
      with:
        path: build
        retention-days: 1
```

### Deploy Job
```yaml
deploy:
  name: Deploy to GitHub Pages
  needs: build
  if: github.ref == 'refs/heads/main'
  permissions:
    pages: write
    id-token: write
  environment:
    name: github-pages
    url: ${{ steps.deployment.outputs.page_url }}
  runs-on: ubuntu-latest
  steps:
    - name: Deploy
      id: deployment
      uses: actions/deploy-pages@v4

    - name: Notify Success
      if: success()
      run: |
        echo "Deployment successful!"
        echo "Site available at: ${{ steps.deployment.outputs.page_url }}"

    - name: Lighthouse Check
      if: success()
      uses: treosh/lighthouse-ci-action@v10
      with:
        configPath: '.lighthouserc.json'
        uploadArtifacts: true
        temporaryPublicStorage: true
```

## Deployment Configuration Schema

### Required Configuration
```typescript
interface DeploymentConfig {
  /** Site URL */
  url: string;

  /** Base URL for subdirectory deployment */
  baseUrl: string;

  /** GitHub organization or username */
  organizationName: string;

  /** Repository name */
  projectName: string;

  /** Deployment branch */
  deploymentBranch: string;

  /** Custom domain (optional) */
  customDomain?: string;
}
```

### Environment-Specific Values
```typescript
const environments = {
  production: {
    url: 'https://[username].github.io',
    baseUrl: '/humanoid_robotic_book/',
    organizationName: '[username]',
    projectName: 'humanoid_robotic_book',
    deploymentBranch: 'gh-pages'
  },
  staging: {
    url: 'https://[username].github.io',
    baseUrl: '/humanoid_robotic_book-staging/',
    organizationName: '[username]',
    projectName: 'humanoid_robotic_book',
    deploymentBranch: 'staging'
  }
};
```

## Build Process

### Pre-build Checks
1. **Configuration Validation**
   - Verify `docusaurus.config.js` syntax
   - Check required fields present
   - Validate baseUrl matches repository

2. **Content Validation**
   - Check all Markdown files have frontmatter
   - Validate required fields in frontmatter
   - Check for broken internal links

3. **Dependency Check**
   - Ensure all dependencies installed
   - Check for security vulnerabilities
   - Validate package.json scripts

### Build Steps
1. **Clean Previous Build**
   ```bash
   rm -rf build/
   rm -rf .docusaurus/
   ```

2. **Generate Static Site**
   ```bash
   NODE_ENV=production npm run build
   ```

3. **Post-build Optimizations**
   ```bash
   # Gzip compression (handled by GitHub Pages)
   # Sitemap generation (handled by Docusaurus)
   # Asset optimization (handled by build process)
   ```

### Build Artifacts
```
build/
├── assets/           # Static assets (CSS, JS, images)
├── docs/             # Generated documentation pages
├── index.html        # Landing page
├── 404.html          # Error page
├──sitemap.xml        # Sitemap for SEO
└──manifest.json      # PWA manifest
```

## Deployment Verification

### Health Checks
```yaml
- name: Health Check
  run: |
    # Check if key files exist
    test -f build/index.html
    test -f build/404.html
    test -f build/sitemap.xml

    # Check if build is under size limit
    du -sh build/ # Should be < 50MB

    # Check for console errors in critical pages
    npm run test-build || true
```

### Lighthouse CI Configuration
```json
{
  "ci": {
    "collect": {
      "numberOfRuns": 3,
      "startServerCommand": "npm run serve",
      "startServerReadyPattern": "Server running",
      "url": ["http://localhost:3000"]
    },
    "assert": {
      "assertions": {
        "categories:performance": ["warn", {"minScore": 0.9}],
        "categories:accessibility": ["error", {"minScore": 0.95}],
        "categories:best-practices": ["warn", {"minScore": 0.9}],
        "categories:seo": ["warn", {"minScore": 0.9}]
      }
    },
    "upload": {
      "target": "temporary-public-storage"
    }
  }
}
```

## Rollback Strategy

### Automatic Rollback Triggers
- Build failure
- Lighthouse score below 80
- Critical 404 errors detected

### Manual Rollback Process
```bash
# 1. Identify last working commit
git log --oneline -10

# 2. Revert to working commit
git revert <commit-hash>

# 3. Push revert
git push origin main

# 4. Monitor deployment
# GitHub Actions will automatically deploy the reverted version
```

## Performance Targets

### Build Performance
- Build time: < 3 minutes
- Artifact size: < 50MB total
- Bundle size: < 500KB gzipped

### Runtime Performance
- First Contentful Paint: < 1.5s
- Largest Contentful Paint: < 2.5s
- Time to Interactive: < 3s
- Cumulative Layout Shift: < 0.1

### Lighthouse Scores
- Performance: > 90
- Accessibility: > 95
- Best Practices: > 90
- SEO: > 90

## Security Considerations

### Build Security
- No secrets in client-side code
- Content Security Policy headers
- Subresource Integrity checks
- Dependency scanning

### Deployment Security
- GitHub Actions secrets management
- Branch protection rules
- Require review for deployment changes
- Audit trail for all deployments

## Monitoring and Observability

### Build Monitoring
```yaml
- name: Build Metrics
  run: |
    # Log build metrics
    echo "Build time: $(($SECONDS / 60)) minutes"
    echo "Bundle size: $(du -sh build/assets/)"

    # Check for warnings
    if [ -f build/.buildinfo.json ]; then
      cat build/.buildinfo.json
    fi
```

### Runtime Monitoring
- Google Analytics integration
- GitHub Pages traffic analytics
- Error tracking (if implemented)
- Performance monitoring

## Environment Management

### Development
```bash
npm start
# Runs on http://localhost:3000
# Hot reload enabled
# No build optimizations
```

### Staging
```bash
npm run build
npm run serve
# Simulates production environment
# For final testing before deployment
```

### Production
- Deployed via GitHub Actions
- URL: https://[username].github.io/humanoid_robotic_book/
- Optimized builds
- CDN caching via GitHub Pages

## Troubleshooting

### Common Issues
1. **404 Errors**
   - Check baseUrl configuration
   - Verify repository name matches
   - Ensure proper file structure

2. **Build Failures**
   - Check Node.js version (require 18+)
   - Clear cache: `npm cache clean --force`
   - Delete node_modules and reinstall

3. **Deployment Failures**
   - Verify GitHub Pages enabled in settings
   - Check permissions for Actions
   - Ensure correct branch name

### Debug Commands
```bash
# Debug build
DEBUG=docusaurus:* npm run build

# Check configuration
npx docusaurus config

# Validate content
npx docusaurus write-translations --locale en
```

## Maintenance

### Regular Tasks
- Update dependencies monthly
- Review Lighthouse scores weekly
- Check for broken links monthly
- Backup content and configuration

### Update Process
```bash
# 1. Update dependencies
npm update

# 2. Test locally
npm run build && npm run serve

# 3. Update documentation
npm run write-docs

# 4. Commit and push
git add .
git commit -m "chore: update dependencies"
git push origin main
```

## Deployment Checklist

### Pre-deployment
- [ ] All tests passing
- [ ] Lighthouse score > 90
- [ ] Content reviewed
- [ ] Configuration validated
- [ ] Dependencies updated

### Post-deployment
- [ ] Site loads correctly
- [ ] No 404 errors on critical pages
- [ ] Forms and links working
- [ ] Performance metrics met
- [ ] Accessibility verified

### Rollback Preparation
- [ ] Previous commit hash noted
- [ ] Backup of current version
- [ ] Communication plan ready
- [ ] Rollback procedure documented