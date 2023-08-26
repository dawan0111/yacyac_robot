import { Module } from '@nestjs/common';
import { ConfigModule, ConfigService } from '@nestjs/config';
import { TypeOrmModule } from '@nestjs/typeorm';
import { AppController } from './app.controller';
import { AppService } from './app.service';
import { UsersModule } from './users/users.module';
import { HistorysModule } from './historys/historys.module';
import { RobotsModule } from './robots/robots.module';

@Module({
  imports: [
    ConfigModule.forRoot({
      isGlobal: true,
    }),
    TypeOrmModule.forRootAsync({
      inject: [ConfigService],
      useFactory: (configService: ConfigService) => ({
        type: 'mysql',
        host: configService.get('DB_HOST'),
        port: +configService.get('DB_PORT'),
        username: configService.get('DB_USERNAME'),
        password: configService.get('DB_PASSWORD'),
        database: configService.get('DB_NAME'),
        entities: ['dist/**/**/*.entity{.ts,.js}'],
        autoLoadEntities: true,
        synchronize: Boolean(configService.get('DB_SYNCHRONIZE') === 'Y'),
      }),
    }),
    UsersModule,
    HistorysModule,
    RobotsModule,
  ],
  controllers: [AppController],
  providers: [AppService],
})
export class AppModule {}
