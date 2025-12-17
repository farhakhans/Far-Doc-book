import React from 'react';
import PropTypes from 'prop-types';
import Tabs from '@theme/Tabs';
import TabItem from '@theme/TabItem';

/**
 * Component for displaying ROS 2 code examples in multiple languages
 */
export default function RosCodeExample({ title, children, lang="python" }) {
  return (
    <div className="ros-code-block">
      {title && <div className="code-block-title">{title}</div>}
      <Tabs groupId="programming-language" defaultValue={lang}>
        <TabItem value="python" label="Python">
          <pre><code className={`language-python`}>{children}</code></pre>
        </TabItem>
        <TabItem value="cpp" label="C++">
          <pre><code className={`language-cpp`}>{children}</code></pre>
        </TabItem>
      </Tabs>
    </div>
  );
}

RosCodeExample.propTypes = {
  title: PropTypes.string,
  lang: PropTypes.string,
  children: PropTypes.node.isRequired
};