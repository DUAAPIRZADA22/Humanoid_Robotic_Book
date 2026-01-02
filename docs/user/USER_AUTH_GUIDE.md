# User Authentication Guide

This guide explains how to use the authentication features of the Humanoid Robotics Textbook.

---

## Creating an Account

### Why Create an Account?

Creating an account allows you to:

- **Access the AI Chat Tutor** - Ask questions and get personalized help
- **Save Your Progress** - Track your learning journey
- **Customize Your Experience** - Set preferences for the textbook
- **Sync Across Devices** - Access your account from anywhere

### Sign Up Steps

1. Click the **"Sign In"** button in the top-right corner of any page
2. Click the **"Create account"** link at the bottom of the modal
3. Fill in the registration form:
   - **Email** - Your email address
   - **Password** - Choose a secure password (see requirements below)
   - **Full Name** - Your name (optional)
4. Click **"Sign Up"**
5. You'll be automatically signed in after registration

### Password Requirements

Your password must:
- Be at least **8 characters** long
- Contain at least **one uppercase letter** (A-Z)
- Contain at least **one lowercase letter** (a-z)
- Contain at least **one number** (0-9)

**Example secure password:** `Robotics2024!`

---

## Signing In

### Standard Sign In

1. Click the **"Sign In"** button
2. Enter your email and password
3. Click **"Sign In"**

### Keep Me Signed In

When you sign in, your session is saved securely in your browser. You'll stay signed in for 7 days before needing to sign in again.

---

## Resetting Your Password

### Forgot Password?

If you've forgotten your password:

1. Click **"Sign In"**
2. Click **"Forgot password?"** link
3. Enter your email address
4. Click **"Send Reset Link"**
5. Check your email for a reset link (or check the console in development mode)
6. Click the link in the email
7. Enter your new password
8. Click **"Reset Password"**
9. Sign in with your new password

### Password Reset Tips

- Reset links expire after 1 hour
- After resetting, you can use your new password immediately
- You can request another reset link if the first one expires

---

## Managing Your Profile

### Accessing Profile Settings

1. Click on your email address in the top-right corner (user menu)
2. Click **"Profile Settings"**

### Updating Your Information

**Change Your Name:**
1. In Profile Settings, edit the "Full Name" field
2. Click **"Update Profile"**

**Change Your Email:**
1. In Profile Settings, edit the "Email" field
2. Enter your current password to confirm the change
3. Click **"Update Profile"**
4. You'll need to use your new email for future sign ins

### Deleting Your Account

If you no longer want your account:

1. Contact support or use the account deletion option in settings
2. Note: Your data will be permanently removed after a retention period
3. You cannot recover a deleted account

---

## Using the AI Chat Tutor

### Accessing Chat

The AI Chat Tutor is available on content pages when you're signed in.

1. Navigate to any textbook chapter
2. Look for the chat widget (usually in the bottom-right corner)
3. Type your question and press Enter or click Send

### Chat Features

- **Ask Questions** - Get help understanding robotics concepts
- **Select Text** - Highlight text in the textbook and click "Ask AI" for context-aware help
- **Streaming Responses** - See answers appear in real-time
- **Conversation History** - Your chat history is saved during your session

### Chat Tips

- Be specific with your questions
- Use the text selection feature for textbook-related questions
- The AI has access to the textbook content for accurate answers

---

## Signing Out

### Manual Sign Out

1. Click your email address in the top-right corner
2. Click **"Sign Out"**
3. You'll be returned to the homepage as a guest

### When to Sign Out

Sign out when you're using a:
- Public or shared computer
- Device that isn't yours
- Browser you don't usually use

### Signing Out Clears

- Your authentication token
- Access to the AI Chat Tutor
- Profile-specific settings

---

## Privacy and Security

### How Your Data Is Protected

- **Passwords** - Never stored in plain text, securely hashed with bcrypt
- **Communication** - Encrypted with HTTPS in production
- **Tokens** - JWT tokens expire after 7 days
- **Storage** - Encrypted localStorage for session persistence

### Your Privacy

- Your email is only used for authentication and password reset
- We don't share your data with third parties
- You can request account deletion at any time
- Chat conversations are processed securely and not used for training

### Best Practices

- **Choose a strong password** that you don't use elsewhere
- **Don't share your password** with anyone
- **Sign out** on shared devices
- **Keep your email updated** so you can receive password reset links

---

## Troubleshooting

### "Invalid email or password"

- Double-check your email for typos
- Verify your password is correct
- Try resetting your password if you've forgotten it
- Make sure Caps Lock is off

### "Email already registered"

- You already have an account with this email
- Try signing in instead of registering
- Use password reset if you've forgotten your password

### "Password too weak"

- Your password must be at least 8 characters
- Include uppercase and lowercase letters
- Add at least one number
- Try adding a special character for extra strength

### "Session expired"

- Your sign-in session has expired (7 days)
- Simply sign in again to continue
- Your data is safe and intact

### Chat widget shows "Sign in required"

- You need to be signed in to use the AI Chat Tutor
- Click the sign in prompt that appears
- After signing in, the chat will work normally

---

## Mobile Experience

### Signing In on Mobile

The authentication experience is optimized for mobile:

- Touch-friendly modals and buttons
- Responsive design for all screen sizes
- Easy-to-tap form fields

### Saving Your Session

- Your session is saved between app closes
- You'll stay signed in for 7 days
- Use device biometrics if available for extra security

---

## Getting Help

### Common Issues

| Issue | Solution |
|-------|----------|
| Can't sign in | Reset your password |
| Didn't receive reset email | Check spam folder, request again |
| Password rejected | Check password requirements |
| Chat not working | Make sure you're signed in |

### Contact Support

If you need help:

1. Check this guide first
2. Try the troubleshooting section above
3. Contact support through the website
4. Include your email (so we can look up your account)

---

## Tips for New Users

### First Time Sign In

1. **Bookmark the site** for easy access
2. **Complete your profile** with your full name
3. **Try the chat** on a chapter to see how it works
4. **Explore chapters** using the navigation

### Getting the Most Out of Your Account

- **Use text selection** - Highlight confusing concepts and ask the AI
- **Ask follow-up questions** - The chat maintains context
- **Explore at your own pace** - Your progress is saved
- **Return often** - The AI learns from textbook content

---

## FAQ

**Q: Do I need to create an account to read the textbook?**
A: No, you can read all content without an account. An account is only required for the AI Chat Tutor.

**Q: Is my chat history saved?**
A: Chat history is saved during your session but not permanently stored. Start a new chat each session for fresh context.

**Q: Can I change my email later?**
A: Yes, go to Profile Settings and update your email. You'll need to confirm with your current password.

**Q: What if I forget my password?**
A: Use the "Forgot password?" link on the sign in page to reset it via email.

**Q: How long do I stay signed in?**
A: Your session lasts 7 days. After that, you'll need to sign in again.

**Q: Is the AI Chat Tutor free?**
A: Yes, it's included with your free account. Some rate limits may apply.

**Q: Can I use my account on multiple devices?**
A: Yes! You can sign in on any device. Your preferences sync across devices.

---

## Security Notice

**Important:** We will never ask for your password via email. If you receive such an email, it's a phishing attempt. Only enter your password on the official website sign in page.
