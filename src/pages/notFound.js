import React from 'react';
import Layout from '@theme/Layout';
import useDocusaurusContext from '@docusaurus/useDocusaurusContext';
import {Redirect} from '@docusaurus/router';

export default function NotFound() {
  const {siteConfig} = useDocusaurusContext();
  return (
    <Layout title={`Page Not Found - ${siteConfig.title}`} description="Page not found">
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--6 col--offset-3">
            <h1 className="text--center">Page Not Found</h1>
            <p className="text--center">
              We couldn't find the page you were looking for.
            </p>
            <p className="text--center">
              <a className="button button--primary button--lg" href="/">
                Go to Homepage
              </a>
            </p>
            <p className="text--center margin-top--lg">
              Or browse the <a href="/docs/intro">Introduction</a> to get started with the Physical AI & Humanoid Robotics book.
            </p>
          </div>
        </div>
      </div>
    </Layout>
  );
}