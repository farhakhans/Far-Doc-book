import React from 'react';
import OriginalDocItemLayout from '@theme-original/DocItem/Layout';

// Custom layout for documentation items that can add book-specific elements
export default function DocItemLayout(props) {
  return (
    <>
      <OriginalDocItemLayout {...props} />
      {/* Add a feedback section at the bottom of each doc page */}
      <div className="container margin-vert--lg">
        <div className="row">
          <div className="col col--8 col--offset-2">
            <hr />
            <h3>Feedback</h3>
            <p>Was this content helpful for your Physical AI & Robotics learning journey?</p>
            <div className="button-group">
              <button className="button button--outline button--success margin-right--sm">Yes</button>
              <button className="button button--outline button--secondary">No</button>
            </div>
          </div>
        </div>
      </div>
    </>
  );
}