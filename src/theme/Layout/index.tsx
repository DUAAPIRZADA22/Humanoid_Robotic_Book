import React from 'react';
import Layout from '@theme-original/Layout';
import { AuthProvider } from '../../contexts/AuthContext';
import ChatWidget from '../../components/ChatWidget';

export default function LayoutWrapper(props) {
  return (
    <AuthProvider>
      <Layout {...props} />
      <ChatWidget />
    </AuthProvider>
  );
}