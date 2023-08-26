import { PartialType } from '@nestjs/mapped-types';
import { CreateRobotDto } from './create-robot.dto';

export class UpdateRobotDto extends PartialType(CreateRobotDto) {}
