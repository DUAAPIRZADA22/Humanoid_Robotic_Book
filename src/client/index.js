/**
 * Docusaurus client entry point
 * @fileoverview Wraps the entire app with AuthProvider
 */

import React from 'react';
import { AuthProvider } from '../contexts/AuthContext';

export default function Root({ children }) {
  return <AuthProvider>{children}</AuthProvider>;
}
