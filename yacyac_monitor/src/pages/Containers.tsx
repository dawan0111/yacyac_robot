import { Button, Input, Modal, Space, Table, Tag } from "antd";
import type { ColumnsType } from "antd/es/table";
import { useQuery } from "react-query";
import fetcher from "../apis/fetcher";
import dayjs from "dayjs";
import { useState } from "react";
import { Controller, useForm } from "react-hook-form";
import { PillData } from "../types";

const pillRender = (value: number) => <>{value} 개</>;

export default function Containers() {
  const { handleSubmit, control, setValue } = useForm();
  const onSubmit = (data: any) => console.log(data);
  const [open, setOpen] = useState(false);

  const showModal = () => {
    setOpen(true);
  };

  const hideModal = () => {
    setOpen(false);
  };

  const { data, error, isLoading } = useQuery<PillData>("robot/1", () =>
    fetcher<PillData>("/robots/1")
  );

  const columns: ColumnsType<{
    name: string;
    pill: number;
  }> = [
    {
      title: "약 이름",
      dataIndex: "name",
      key: "name",
    },
    {
      title: "수량",
      dataIndex: "pill",
      key: "pill",
      render: pillRender,
    },
    {
      title: "-",
      render: (_, record, index) => (
        <>
          <Button
            type="primary"
            onClick={() => {
              setValue("id", index + 1);
              setValue("name", record.name);
              setValue("pill", record.pill);

              showModal();
            }}
          >
            정보 변경
          </Button>
        </>
      ),
    },
  ];

  const columnData = data
    ? Array(8)
        .fill(0)
        .map((_, index) => {
          const pillIndex = `pill${index + 1}`;
          return {
            pill: data[pillIndex as keyof PillData],
            name: `${index + 1}번 약`,
          };
        })
    : [];

  return (
    <>
      <Table columns={columns} dataSource={columnData} />
      <Modal
        title="약통 정보 변경"
        open={open}
        onOk={hideModal}
        onCancel={hideModal}
        okText="변경"
        cancelText="취소"
      >
        <form onSubmit={handleSubmit(onSubmit)}>
          <Space direction="vertical" style={{ width: "100%" }}>
            <div>
              <label htmlFor="name">약 이름</label>
              <Controller
                name="name"
                defaultValue=""
                control={control}
                render={({ field }) => <Input id="name" {...field} />}
              />
            </div>
            <div>
              <label htmlFor="name">약 갯수</label>
              <Controller
                name="pill"
                defaultValue="0"
                control={control}
                render={({ field }) => (
                  <Input id="pill" {...field} type="number" />
                )}
              />
            </div>
          </Space>
        </form>
      </Modal>
    </>
  );
}
