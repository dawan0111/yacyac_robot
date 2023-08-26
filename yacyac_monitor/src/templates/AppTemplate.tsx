import React, { useState } from "react";
import {
  DesktopOutlined,
  FileOutlined,
  PieChartOutlined,
  TeamOutlined,
  UserOutlined,
} from "@ant-design/icons";
import type { MenuProps } from "antd";
import { Layout, Menu, theme } from "antd";
import { Link } from "react-router-dom";
import Clock from "../components/Clock";

const { Header, Content, Footer, Sider } = Layout;

type MenuItem = Required<MenuProps>["items"][number];

function getItem(
  label: React.ReactNode,
  key: React.Key,
  icon?: React.ReactNode,
  children?: MenuItem[]
): MenuItem {
  return {
    key,
    icon,
    children,
    label,
  } as MenuItem;
}

const items: MenuItem[] = [
  getItem(<Link to="/">메인 화면</Link>, "1", <PieChartOutlined />),
  getItem(<Link to="/containers">약통 관리</Link>, "2", <DesktopOutlined />),
  getItem(<Link to="/historys">배급 내역</Link>, "3", <DesktopOutlined />),
];

const AppTemplate: React.FC<{
  children: React.ReactNode;
}> = ({ children }) => {
  const [collapsed, setCollapsed] = useState(false);
  const {
    token: { colorBgContainer },
  } = theme.useToken();

  return (
    <Layout style={{ minHeight: "100vh" }}>
      <Sider
        collapsible
        collapsed={collapsed}
        onCollapse={(value) => setCollapsed(value)}
      >
        <div
          className="logo"
          style={{
            textAlign: "center",
            fontSize: "2.5rem",
            color: "#fff",
            fontWeight: "bold",
            fontFamily: "Black Ops One",
            margin: ".5rem 0 1.5rem 0",
          }}
        >
          YACYAC
        </div>
        <Menu
          theme="dark"
          defaultSelectedKeys={["1"]}
          mode="inline"
          items={items}
          onChange={(event) => {
            console.log(event);
          }}
        />
      </Sider>
      <Layout>
        <Header
          style={{
            padding: "0 2rem",
            background: colorBgContainer,
            display: "flex",
            alignItems: "center",
            justifyContent: "flex-end",
          }}
        >
          <Clock />
        </Header>
        <Content style={{ margin: "16px" }}>{children}</Content>
        <Footer style={{ textAlign: "center" }}>
          YacYac ©2023 Created by YacYac
        </Footer>
      </Layout>
    </Layout>
  );
};

export default AppTemplate;
