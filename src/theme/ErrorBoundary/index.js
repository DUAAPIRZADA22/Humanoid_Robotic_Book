import React from 'react';
import clsx from 'clsx';

export default function ErrorBoundaryFallback({error, tryAgain}) {
  return (
    <div className="flex min-h-screen items-center justify-center bg-slate-900 px-4">
      <div className="max-w-md rounded-xl glass p-8 text-center">
        <div className="mb-4 text-6xl">😵</div>
        <h1 className="mb-4 text-2xl font-bold text-white">
          Oops! Something went wrong
        </h1>
        <p className="mb-6 text-slate-400">
          We're sorry, but something unexpected happened. Please try refreshing the page.
        </p>
        {error && (
          <details className="mb-6 text-left">
            <summary className="cursor-pointer text-sm text-slate-500 hover:text-slate-400">
              Error details
            </summary>
            <pre className="mt-2 overflow-auto rounded-lg bg-slate-800 p-4 text-xs text-red-400">
              {error.message}
              {error.stack}
            </pre>
          </details>
        )}
        <button
          type="button"
          className="button button--primary"
          onClick={tryAgain}
        >
          Try Again
        </button>
      </div>
    </div>
  );
}