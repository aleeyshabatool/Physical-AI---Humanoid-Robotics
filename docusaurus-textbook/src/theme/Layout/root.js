import React from 'react';
import Layout from '@theme-original/Layout';
import ChatbotButton from '@site/src/components/ChatbotButton';

export default function LayoutWrapper(props) {
  return (
    <>
                  <Layout {...props}>
                    {props.children}
                   
                  </Layout>
                  <ChatbotButton />
                </>  );
} 