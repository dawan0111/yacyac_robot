import { Col, Row, theme } from "antd";
import styled from "styled-components";
import RobotMap from "../components/RobotMap";
import Title from "antd/es/typography/Title";
import StatusCard from "../components/StatusCard";
import { useQuery } from "react-query";
import fetcher from "../apis/fetcher";
import { PillData } from "../types";
import ROSLIB, { Ros } from "roslib";
import { useEffect, useRef, useState } from "react";
import { mat4, quat, vec3 } from "gl-matrix";
import { createSE3Matrix } from "../utils/math";
import map from "../assets/configs/map.json";

const Card = styled.div`
  position: relative;
  width: 100%;
  height: 100%;
  background: #fff;
  padding: 16px;
  border-radius: 4px;
  overflow: hidden;
`;

const LargeInfoText = styled.div`
  font-size: 3rem;
  margin-top: 3.6rem;
  font-weight: bold;
  text-align: center;
`;

const throttle = function (callback: any, waitMs: number) {
  let wait = false;
  return function (...args: any) {
    if (!wait) {
      wait = true;
      setTimeout(() => {
        wait = false;
        callback(...args);
      }, waitMs);
    }
  };
};

export default function Dashboard() {
  const {
    token: { colorPrimary },
  } = theme.useToken();
  const connection = useRef(false);

  const { data } = useQuery<PillData>("robot/1", () =>
    fetcher<PillData>("/robots/1")
  );
  const { data: historyStat } = useQuery<PillData[]>("historys/stat", () =>
    fetcher<PillData[]>("/historys/stat")
  );

  const [map2OdomTf, setMap2OdomTf] = useState<mat4>(mat4.create());
  const [odom2baseTf, setOdom2baseTf] = useState<mat4>(mat4.create());
  const map2baseTf = mat4.create();

  mat4.multiply(map2baseTf, map2OdomTf, odom2baseTf);

  const openRosSocket = () => {
    connection.current = true;

    const tfCallback = throttle((msg: any) => {
      const { x, y, z } = msg.transforms[0].transform.translation;
      const {
        w: r_w,
        x: r_x,
        y: r_y,
        z: r_z,
      } = msg.transforms[0].transform.rotation;
      const translation: vec3 = vec3.fromValues(x, y, z);
      const quaternion: quat = quat.fromValues(r_x, r_y, r_z, r_w);

      const se3Matrix: mat4 = createSE3Matrix(translation, quaternion);

      if (msg.transforms[0].child_frame_id === "odom") {
        setMap2OdomTf(se3Matrix);
      } else if (msg.transforms[0].child_frame_id === "base_footprint") {
        setOdom2baseTf(se3Matrix);
      }
    }, 60);

    const ros = new Ros({
      url: "ws://192.168.0.40:9090",
    });

    ros.on("connection", function () {
      console.log("Connection made!");
    });

    const tfTopic = new ROSLIB.Topic({
      ros: ros,
      name: "/tf",
      messageType: "tf2_msgs/msg/TFMessage",
    });

    tfTopic.subscribe(tfCallback);

    return () => {
      // tfTopic.unsubscribe(tfCallback);
      // robotStatusTopic.unsubscribe(robotStatusCallback);
      // odomTopic.unsubscribe(robotOdomPoseCallback);
    };
  };
  useEffect(() => {
    if (!connection.current) {
      return openRosSocket();
    }
  }, []);

  return (
    <>
      <Row gutter={[16, 16]}>
        <Col span={24}>
          <Card
            style={{
              background: "linear-gradient(180deg, #001529, #333)",
              height: "600px",
            }}
          >
            <RobotMap
              imageSrc="/one.png"
              robotPose={map2baseTf}
              origins={map.origins}
              markers={map.points}
            />
          </Card>
        </Col>
        <Col span={16}>
          <Card>
            <Title level={3} style={{ margin: "0 0 16px 0" }}>
              알약 통 현황
            </Title>
            {data && (
              <Row gutter={[24, 24]}>
                {Array(8)
                  .fill(0)
                  .map((i, index) => (
                    <Col span={6} key={index}>
                      <StatusCard
                        title={<>{index + 1}번 약</>}
                        content={
                          <>
                            <span style={{ color: colorPrimary }}>
                              {data[`pill${index + 1}` as keyof PillData]}
                            </span>{" "}
                            개
                          </>
                        }
                      />
                    </Col>
                  ))}
              </Row>
            )}
          </Card>
        </Col>
        <Col span={8}>
          <Card>
            <Title level={3} style={{ margin: "0 0 16px 0" }}>
              알약 배급 횟수
            </Title>
            <LargeInfoText>
              <span style={{ color: colorPrimary }}>
                {historyStat?.toLocaleString()}
              </span>
              번
            </LargeInfoText>
          </Card>
        </Col>
      </Row>
    </>
  );
}
