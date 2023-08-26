import { Injectable } from '@nestjs/common';
import { InjectRepository } from '@nestjs/typeorm';
import { Repository } from 'typeorm';
import { Robot } from './entities/robot.entity';
import { CreateRobotDto } from './dto/create-robot.dto';
import { UpdateRobotDto } from './dto/update-robot.dto';
import { SupplyUpdateDTO } from './dto/supply-update.dto';

@Injectable()
export class RobotsService {
  constructor(
    @InjectRepository(Robot)
    private readonly robotRepository: Repository<Robot>,
  ) {}

  async create(robotData: CreateRobotDto): Promise<Robot> {
    const robot = this.robotRepository.create(robotData);
    return await this.robotRepository.save(robot);
  }

  async findAll(): Promise<Robot[]> {
    return await this.robotRepository.find();
  }

  async findOne(id: number): Promise<Robot> {
    return await this.robotRepository.findOne({ where: { id } });
  }

  async update(id: number, robotData: UpdateRobotDto): Promise<Robot> {
    await this.robotRepository.update(id, robotData);
    return await this.robotRepository.findOne({ where: { id } });
  }

  async remove(id: number): Promise<void> {
    await this.robotRepository.delete(id);
  }

  async supplyUpdate(supplyData: SupplyUpdateDTO): Promise<Robot> {
    const id = 1;
    const robot = await this.robotRepository.findOne({ where: { id } });

    robot.pill1 -= supplyData.pill1;
    robot.pill2 -= supplyData.pill2;
    robot.pill3 -= supplyData.pill3;
    robot.pill4 -= supplyData.pill4;
    robot.pill5 -= supplyData.pill5;
    robot.pill6 -= supplyData.pill6;
    robot.pill7 -= supplyData.pill7;
    robot.pill8 -= supplyData.pill8;

    await this.robotRepository.update(1, {
      pill1: robot.pill1,
      pill2: robot.pill2,
      pill3: robot.pill3,
      pill4: robot.pill4,
      pill5: robot.pill5,
      pill6: robot.pill6,
      pill7: robot.pill7,
      pill8: robot.pill8,
    });

    return robot;
  }
}
