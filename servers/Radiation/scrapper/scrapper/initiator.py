'''from scrapy.crawler import CrawlerProcess
from scrapy.utils.project import get_project_settings

process =  CrawlerProcess(get_project_settings())

process.crawl('forspef',domain = 'http://tromos.space.noa.gr/forspef/modules/')
process.start()

'''
from twisted.internet import reactor
import scrapy
from scrapy.crawler import CrawlerRunner
from scrapy.utils.log import configure_logging
from forspef_scrapper import ForspefScrapper
from scrapy.settings import Settings
from scrapy.utils.project import get_project_settings


configure_logging({'LOG_FORMAT': '%(levelname)s: %(message)s'})

runner = CrawlerRunner(get_project_settings())
d = runner.crawl(ForspefScrapper)
d.addBoth(lambda _: reactor.stop())
reactor.run()
'''
from twisted.internet import reactor
from scrapy.crawler import Crawler
from scrapy import log, signals
from scrapy.settings import Settings
from scrapy.utils.project import get_project_settings
from spiders.forspef_scrapper import ForspefScrapper
import os 

spider = ForspefScrapper()

settings = Settings()
os.environ['SCRAPY_SETTINGS_MODULE'] = 'settings'
settings_module_path = os.environ['SCRAPY_SETTINGS_MODULE']
settings.setmodule(settings_module_path, priority='project')
crawler = Crawler(ForspefScrapper,settings)

crawler.signals.connect(reactor.stop, signal=signals.spider_closed)
#crawler.configure()
crawler.crawl(spider)
#crawler.start()
#log.start(loglevel=log.INFO)
reactor.run()
'''
