import { Module } from '@nestjs/common';
import { HistorysService } from './historys.service';
import { HistorysController } from './historys.controller';
import { TypeOrmModule } from '@nestjs/typeorm';
import { History } from './entities/history.entity';
import { RobotsModule } from 'src/robots/robots.module';

@Module({
  imports: [TypeOrmModule.forFeature([History]), RobotsModule],
  controllers: [HistorysController],
  providers: [HistorysService],
})
export class HistorysModule {}
