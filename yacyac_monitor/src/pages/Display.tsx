// @ts-nocheck

import { useEffect, useRef, useState } from "react";
import styled, { createGlobalStyle } from "styled-components";
import ROSLIB, { Ros } from "roslib";
import { Button, Progress } from "antd";
import Lottie from "lottie-react";

import { useQuery, useMutation } from "react-query";
import fetcher from "../apis/fetcher";
import { saveHistory } from "../apis/History";

import pillAnimation from "../lotties/pill.json";
import eyesAnimation from "../lotties/eyes.json";

const DisplayStyled = styled.div`
  width: 100vw;
  height: 100vh;
  background: #000;
`;

const QRScreenStyled = styled.div`
  display: flex;
  flex-direction: column;
  position: realtive;
  width: 100%;
  height: 100%;
  background: #fff;
  color: #000;
  .info {
    position: absolute;
    z-index: 2;
    width: 100%;
    top: 2rem;
    text-align: center;
    color: #fff;
    font-size: 2rem;
    font-weight: bold;
    text-shadow: 2px 4px 2px gray;
  }

  .image-wrap {
    display: flex;
    justify-content: center;
    width: 100%;
    flex: 1;
    background: #000;
    transform: scaleX(-1);

    img {
      height: 100%;
    }
  }
`;
const ActionPopupStyled = styled.div<{
  open: boolean;
}>`
  ${(props) =>
    props.open
      ? `
    opacity: 1;
    transform: translateY(0);
  `
      : `
    opacity: 0;
    transform: translateY(4rem);
  `}
  display: flex;
  flex-direction: column;
  align-items: center;
  position: absolute;
  z-index: 2;
  left: 0;
  bottom: 0;
  padding: 1rem;
  background: rgba(255, 255, 255, 0.9);
  transition: 500ms ease;
  width: 100%;
  backdrop-filter: blur(10px);

  border-top-left-radius: 1rem;
  border-top-right-radius: 1rem;

  text-align: center;

  .image {
    width: 5rem;
    height: 5rem;
    background: #000;
    margin-bottom: 1rem;
    font-size: 0;
    img {
      width: 100%;
      height: 100%;
      object-fit: cover;
    }
  }

  .name {
    font-size: 1.4rem;
    font-weight: bold;
  }
  .description {
    font-size: 1.4rem;
  }

  .action-button {
    display: flex;
    width: 100%;
    gap: 1rem;
    margin-top: 1rem;
    button {
      flex: 1;
      padding-top: 0.5rem;
      padding-bottom: 0.5rem;
      font-size: 1.4rem;
      height: auto;
    }
  }
`;

const OutputStyled = styled.div`
  display: flex;
  flex-direction: column;
  align-items: center;
  position: realtive;
  width: 100%;
  height: 100%;
  padding-top: 1rem;
  padding-bottom: 1rem;
  background: #fff;
  color: #000;

  .title {
    font-size: 1.8rem;
    font-weight: bold;
    margin-bottom: 0.5rem;
  }

  .progress-info {
    font-size: 1.2rem;
  }

  .animation {
    margin: 2rem 0;
    width: 100%;
    display: flex;
    justify-content: center;

    > div {
      width: 25%;
    }
  }

  .progress-bar {
    width: 80%;

    .ant-progress-outer,
    .ant-progress-bg {
      height: 1.4rem !important;
    }
  }
`;

const FaceStyled = styled.div`
  display: flex;
  align-items: center;
  justify-content: center;
  width: 100%;
  height: 100%;

  & > div {
    height: 90%;
  }
`;

const GlobalStyled = createGlobalStyle`
  body, html {
    overflow: hidden;
  }
`;

type TScreen = "FACE" | "QR_CODE" | "OUTPUT";

export default function Display() {
  const [screenStatus, setScreenStatus] = useState<TScreen>("FACE");
  const [openConfirm, setOpenConfirm] = useState(false);
  const [detectUserId, setDetectUserId] = useState<number | null>(null);
  const [currentSupplyCount, setCurrentSupplyCount] = useState(0);

  const connection = useRef(false);
  const feedbackTopicRef = useRef(null);
  const imageTag = useRef(null);

  const { mutate: saveHistoryMutate } = useMutation(saveHistory);

  const { data: userInfo, isLoading } = useQuery<PillData>(
    detectUserId ? `users/${detectUserId}` : null,
    () => fetcher<PillData>(`users/${detectUserId}`)
  );

  const totalPill = userInfo
    ? userInfo.pill1 +
      userInfo.pill2 +
      userInfo.pill3 +
      userInfo.pill4 +
      userInfo.pill5 +
      userInfo.pill6 +
      userInfo.pill7 +
      userInfo.pill8
    : 0;

  const openRosSocket = () => {
    connection.current = true;
    const cameraCallback = (msg: any) => {
      if (imageTag.current) {
        const imagedata = `data:image/png;base64,${msg.data}`;
        imageTag.current.src = imagedata;
      }
    };

    const robotStatusCallback = (msg: any) => {
      if (msg.qr_infos.length === 1) {
        setOpenConfirm(true);
        setDetectUserId(msg.qr_infos[0]);
      }
    };

    const modeCallback = (msg: any) => {
      if (msg.data === 0) {
        setScreenStatus("FACE");
        setDetectUserId(null);
      } else if (msg.data === 1) {
        setScreenStatus("QR_CODE");
      } else if (msg.data === 2) {
        setScreenStatus("OUTPUT");
      }
    };

    const supplyFeedbackCallback = (msg: any) => {
      setCurrentSupplyCount(msg.data);
    };

    const ros = new Ros({
      url: "ws://192.168.0.40:9090",
    });

    ros.on("connection", function () {
      console.log("Connection made!");
    });

    const cameraTopic = new ROSLIB.Topic({
      ros,
      name: "/yacyac_top_camera/compressed",
      messageType: "sensor_msgs/msg/CompressedImage",
    });

    const robotStatusTopic = new ROSLIB.Topic({
      ros,
      name: "/qr_node",
      messageType: "yacyac_interface/msg/Qrcode",
    });

    const modeTopic = new ROSLIB.Topic({
      ros,
      name: "/mode",
      messageType: "std_msgs/msg/Int8",
    });

    const feedbackTopic = new ROSLIB.Topic({
      ros,
      name: "/user_feedback",
      messageType: "std_msgs/msg/Bool",
    });

    const supplyFeedbackTopic = new ROSLIB.Topic({
      ros,
      name: "/yacyac/supply/process",
      messageType: "std_msgs/msg/Int8",
    });

    cameraTopic.subscribe(cameraCallback);
    robotStatusTopic.subscribe(robotStatusCallback);
    modeTopic.subscribe(modeCallback);
    supplyFeedbackTopic.subscribe(supplyFeedbackCallback);

    feedbackTopicRef.current = feedbackTopic;

    return () => {
      // robotStatusTopic.unsubscribe(robotStatusCallback);
      // odomTopic.unsubscribe(robotOdomPoseCallback);
    };
  };
  useEffect(() => {
    if (!connection.current) {
      return openRosSocket();
    }
  }, []);

  const saveAcceptHistory = async (isAccept: boolean) => {
    saveHistoryMutate({
      userId: userInfo.id,
      pill1: userInfo.pill1,
      pill2: userInfo.pill2,
      pill3: userInfo.pill3,
      pill4: userInfo.pill4,
      pill5: userInfo.pill5,
      pill6: userInfo.pill6,
      pill7: userInfo.pill7,
      pill8: userInfo.pill8,
      accept: isAccept,
    });
  };

  const onCancelClick = async () => {
    setOpenConfirm(false);
    await saveAcceptHistory(false);
    if (feedbackTopicRef.current) {
      feedbackTopicRef.current.publish({
        data: false,
      });
    }
  };
  const onAcceptClick = async () => {
    setOpenConfirm(false);
    await saveAcceptHistory(true);
    setScreenStatus("OUTPUT");

    if (feedbackTopicRef.current) {
      feedbackTopicRef.current.publish({
        data: true,
      });
    }
  };

  return (
    <DisplayStyled>
      <GlobalStyled />
      {screenStatus === "QR_CODE" && (
        <QRScreenStyled>
          <div className="info">QR을 인식해주세요</div>
          <div className="image-wrap">
            <img id="image" alt="camera" ref={imageTag} />
          </div>
          <ActionPopupStyled open={openConfirm && Boolean(userInfo)}>
            {userInfo && (
              <>
                <div className="image">
                  <img src={userInfo?.photo} alt="환자 얼굴" />
                </div>
                <div className="name">{userInfo?.name}님</div>
                <div className="description">약 배급을 받으시겠습니까?</div>
                <div className="action-button">
                  <Button type="primary" onClick={onAcceptClick}>
                    수락
                  </Button>
                  <Button onClick={onCancelClick}>거부</Button>
                </div>
              </>
            )}
          </ActionPopupStyled>
        </QRScreenStyled>
      )}
      {screenStatus === "OUTPUT" && (
        <OutputStyled>
          <div className="title">약 배급중...</div>
          <div className="progress-info">
            {totalPill}개중 {currentSupplyCount}개 완료 (
            {((totalPill / currentSupplyCount) * 100).toFixed(2)}%)
          </div>
          <div className="animation">
            <Lottie animationData={pillAnimation} loop={true} />
          </div>
          <div className="progress-bar">
            <Progress
              status="active"
              percent={((totalPill / currentSupplyCount) * 100).toFixed(2)}
              strokeColor={{ "0%": "#108ee9", "100%": "#87d068" }}
            />
          </div>
        </OutputStyled>
      )}
      {screenStatus === "FACE" && (
        <FaceStyled>
          <Lottie animationData={eyesAnimation} loop={true} />
        </FaceStyled>
      )}
    </DisplayStyled>
  );
}
