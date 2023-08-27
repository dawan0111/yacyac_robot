import {
  Controller,
  Get,
  Post,
  Body,
  Patch,
  Param,
  Delete,
} from '@nestjs/common';
import { RobotsService } from './robots.service';
import { CreateRobotDto } from './dto/create-robot.dto';
import { UpdateRobotDto } from './dto/update-robot.dto';

@Controller('robots')
export class RobotsController {
  constructor(private readonly robotsService: RobotsService) {}

  @Post()
  async create(@Body() createRobotDto: CreateRobotDto) {
    return await this.robotsService.create(createRobotDto);
  }

  @Get()
  async findAll() {
    return await this.robotsService.findAll();
  }

  @Get(':id')
  async findOne(@Param('id') id: string) {
    return await this.robotsService.findOne(+id);
  }

  @Patch(':id')
  async update(
    @Param('id') id: string,
    @Body() updateRobotDto: UpdateRobotDto,
  ) {
    return await this.robotsService.update(+id, updateRobotDto);
  }

  @Delete(':id')
  async remove(@Param('id') id: string) {
    return await this.robotsService.remove(+id);
  }
}
