import { Module } from '@nestjs/common';
import { RobotsService } from './robots.service';
import { RobotsController } from './robots.controller';
import { TypeOrmModule } from '@nestjs/typeorm';
import { Robot } from './entities/robot.entity';

@Module({
  imports: [TypeOrmModule.forFeature([Robot])],
  controllers: [RobotsController],
  providers: [RobotsService],
  exports: [RobotsService],
})
export class RobotsModule {}
